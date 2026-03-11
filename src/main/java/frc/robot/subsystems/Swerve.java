package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.oldeLandmarks;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

public class Swerve extends TunerSwerveDrivetrain implements Subsystem {

    // ---- Singleton ----
    private static Swerve mySwerve;

    public static Swerve get() {
        if (mySwerve == null) {
            mySwerve = new Swerve();
        }
        return mySwerve;
    }

    // ---- Limelight target tracking ----
    public static double currentTargetID;

    // ---- Sim ----
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    // ---- AdvantageKit / AdvantageScope telemetry state ----
    private SwerveModuleState[] m_lastSetpoints = new SwerveModuleState[] {
        new SwerveModuleState(), new SwerveModuleState(),
        new SwerveModuleState(), new SwerveModuleState()
    };
    private ChassisSpeeds m_lastSetpointSpeeds = new ChassisSpeeds();

    // Kinematics built from module positions (telemetry-only)
    private static final SwerveDriveKinematics kTelemetryKinematics = new SwerveDriveKinematics(
        new Translation2d(TunerConstants.FrontLeft.LocationX,  TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX,   TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX,  TunerConstants.BackRight.LocationY)
    );

    // ---- SysId routines ----
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, Volts.of(4), null,
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null, this
        )
    );

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, Volts.of(7), null,
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null, this
        )
    );

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Math.PI / 6).per(Second), Volts.of(Math.PI), null,
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null, this
        )
    );

    // ---- Drive requests ----
    /** Swerve request to apply during field-centric path following */
    private final SwerveRequest.ApplyFieldSpeeds pathFieldSpeedsRequest = new SwerveRequest.ApplyFieldSpeeds();
    private final PIDController pathXController = new PIDController(10, 0, 0);
    private final PIDController pathYController = new PIDController(10, 0, 0);
    private final PIDController pathThetaController = new PIDController(7, 0, 0);

    // ---- Constructor (private for singleton) ----
    private Swerve() {
        super(
            TunerConstants.DrivetrainConstants,
            0,
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.1, 0.1, 0.1),
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        );

        pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        seedFieldCentric();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    // ---- SysId commands ----
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineTranslation.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineTranslation.dynamic(direction);
    }

    public Command sysIdQuasistaticSteer(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineSteer.quasistatic(direction);
    }

    public Command sysIdDynamicSteer(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineSteer.dynamic(direction);
    }

    public Command sysIdQuasistaticRotation(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineRotation.quasistatic(direction);
    }

    public Command sysIdDynamicRotation(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineRotation.dynamic(direction);
    }

    // ---- Sim thread ----
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {});
    }

    /**
     * Creates a new auto factory for this drivetrain with the given trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
            () -> getState().Pose,
            this::resetPose,
            this::followPath,
            true,
            this,
            trajLogger
        );
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        var pose = getState().Pose;

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += pathXController.calculate(
            pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond += pathYController.calculate(
            pose.getY(), sample.y
        );
        targetSpeeds.omegaRadiansPerSecond += pathThetaController.calculate(
            pose.getRotation().getRadians(), sample.heading
        );

        // Record for AdvantageScope setpoint tabs
        recordSetpointTelemetry(
            kTelemetryKinematics.toSwerveModuleStates(targetSpeeds),
            targetSpeeds
        );

        setControl(
            pathFieldSpeedsRequest.withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }

    @Override
    public void periodic() {
        // Limelight target ID
        currentTargetID = LimelightHelpers.getFiducialID("limelight");

        /*
         * Periodically try to apply the operator perspective.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                if (!m_hasAppliedOperatorPerspective) {
                    seedFieldCentric();
                }
                m_hasAppliedOperatorPerspective = true;
            });
        }

        // ---- AdvantageKit / AdvantageScope telemetry ----
        SwerveDriveState state = getState();

        Logger.recordOutput("RealOutputs/SwerveStates/Measured", state.ModuleStates);
        Logger.recordOutput("RealOutputs/SwerveStates/SetpointsOptimized", state.ModuleTargets);
        Logger.recordOutput("RealOutputs/SwerveStates/Setpoints", m_lastSetpoints);
        Logger.recordOutput("RealOutputs/SwerveChassisSpeeds/Measured", state.Speeds);
        Logger.recordOutput("RealOutputs/SwerveChassisSpeeds/Setpoints", m_lastSetpointSpeeds);
        Logger.recordOutput("RealOutputs/Odometry/Robot", state.Pose);

        for (int i = 0; i < 4; i++) {
            var modState = state.ModuleStates[i];
            Logger.recordOutput("Drive/Module" + i + "/TurnPosition",
                modState.angle.getRadians());
            var module = getModule(i);
            double drivePositionRad = Units.rotationsToRadians(
                module.getDriveMotor().getPosition().getValueAsDouble()
                    / TunerConstants.FrontLeft.DriveMotorGearRatio);
            Logger.recordOutput("Drive/Module" + i + "/DrivePositionRad", drivePositionRad);
            Logger.recordOutput("Drive/Module" + i + "/DriveVelocityRadPerSec",
                Units.rotationsToRadians(
                    module.getDriveMotor().getVelocity().getValueAsDouble()
                        / TunerConstants.FrontLeft.DriveMotorGearRatio));
            Logger.recordOutput("Drive/Module" + i + "/DriveCurrentAmps",
                module.getDriveMotor().getStatorCurrent().getValueAsDouble());
            Logger.recordOutput("Drive/Module" + i + "/TurnCurrentAmps",
                module.getSteerMotor().getStatorCurrent().getValueAsDouble());
            Logger.recordOutput("Drive/Module" + i + "/TurnAppliedVolts",
                module.getSteerMotor().getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Drive/Module" + i + "/DriveAppliedVolts",
                module.getDriveMotor().getMotorVoltage().getValueAsDouble());
        }
    }

    /**
     * Call this whenever you send a new velocity setpoint to the drivetrain so that
     * AdvantageScope can overlay commanded vs. measured on the calibration tabs.
     */
    public void recordSetpointTelemetry(SwerveModuleState[] setpoints, ChassisSpeeds speeds) {
        m_lastSetpoints = setpoints;
        m_lastSetpointSpeeds = speeds;
    }

    /** Returns the current robot pose from the CTRE odometry estimator. */
    public Pose2d getPose() {
        return getState().Pose;
    }

    /** Returns the current measured chassis speeds. */
    public ChassisSpeeds getChassisSpeeds() {
        return getState().Speeds;
    }

    /**
     * Returns the direction from the robot to the hub landmark, relative to the operator forward direction.
     */
    public Rotation2d getAimDirection() {
        Translation2d hubPosition = oldeLandmarks.hubPosition().getTranslation();
        Translation2d robotPosition = getState().Pose.getTranslation();
        return hubPosition.minus(robotPosition).getAngle()
                .rotateBy(getOperatorForwardDirection());
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(timestampSeconds);
    }
}
