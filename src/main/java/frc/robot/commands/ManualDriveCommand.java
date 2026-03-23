package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
import frc.robot.subsystems.Swerve;
import frc.util.DriveInputSmoother;
import frc.util.ManualDriveInput;
import frc.util.Stopwatch;

import static edu.wpi.first.units.Units.MetersPerSecond;

/**
 * Teleop manual drive command for the swerve drivetrain.
 *
 * Handles field-centric driving with manual rotation input and
 * heading-hold behavior after a short delay once rotation input
 * returns to zero.
 */
public class ManualDriveCommand extends Command {
    private enum State {
        IDLING,
        DRIVING_WITH_MANUAL_ROTATION,
        DRIVING_WITH_LOCKED_HEADING
    }

    private static final Time kHeadingLockDelay = Seconds.of(0.75); // time to wait before locking heading
    private static final LinearVelocity kIntakeTriggerSpeedLimit = MetersPerSecond.of(1.0); // speed limit when intake trigger is pressed

    private final Swerve swerve;
    private final DriveInputSmoother inputSmoother;
    private final BooleanSupplier intakeTriggerSupplier;

    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withRotationalDeadband(Driving.kPIDRotationDeadband)
            .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withHeadingPID(5, 0, 0);
    //         .withHeadingPID(0.25, 0, 0);

    private State currentState = State.IDLING;
    private Optional<Rotation2d> lockedHeading = Optional.empty();
    private Stopwatch headingLockStopwatch = new Stopwatch();
    private ManualDriveInput previousInput = new ManualDriveInput();

    public ManualDriveCommand(
            Swerve swerve,
            DoubleSupplier forwardInput,
            DoubleSupplier leftInput,
            DoubleSupplier rotationInput,
            BooleanSupplier intakeTriggerSupplier
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput, rotationInput);
        this.intakeTriggerSupplier = intakeTriggerSupplier;
        addRequirements(swerve);
    }

    public void seedFieldCentric() {
        initialize();
        swerve.seedFieldCentric();
    }

    private LinearVelocity getCurrentSpeedLimit() {
        return intakeTriggerSupplier.getAsBoolean() ? kIntakeTriggerSpeedLimit : Driving.kMaxSpeed;
    }

    public void setLockedHeading(Rotation2d heading) {
        lockedHeading = Optional.of(heading);
        currentState = State.DRIVING_WITH_LOCKED_HEADING;
    }

    private void setLockedHeadingToCurrent() {
        final Rotation2d headingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
        final Rotation2d headingInOperatorPerspective = headingInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        setLockedHeading(headingInOperatorPerspective);
    }

    private void lockHeadingIfRotationStopped(ManualDriveInput input) {
        if (input.hasRotation()) {
            headingLockStopwatch.reset();
            lockedHeading = Optional.empty();
        } else {
            headingLockStopwatch.startIfNotRunning();
            if (headingLockStopwatch.elapsedTime().gt(kHeadingLockDelay)) {
                setLockedHeadingToCurrent();
            }
        }
    }

    @Override
    public void initialize() {
        currentState = State.IDLING;
        lockedHeading = Optional.empty();
        headingLockStopwatch.reset();
        previousInput = new ManualDriveInput();
    }

    @Override
    public void execute() {
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        final LinearVelocity speedLimit = getCurrentSpeedLimit();
        
        if (input.hasRotation()) {
            currentState = State.DRIVING_WITH_MANUAL_ROTATION;
        } else if (input.hasTranslation()) {
            currentState = lockedHeading.isPresent() ? State.DRIVING_WITH_LOCKED_HEADING : State.DRIVING_WITH_MANUAL_ROTATION;
        } else if (previousInput.hasRotation() || previousInput.hasTranslation()) {
            currentState = State.IDLING;
        }
        previousInput = input;

        switch (currentState) {
            case IDLING:
                swerve.setControl(
                        fieldCentricRequest
                                .withVelocityX(0)
                                .withVelocityY(0)
                                .withRotationalRate(0)
                );
                swerve.recordSetpointTelemetry(
                        new SwerveModuleState[]{ new SwerveModuleState(), new SwerveModuleState(),
                                new SwerveModuleState(), new SwerveModuleState() },
                        new ChassisSpeeds()
                );
                break;
            case DRIVING_WITH_MANUAL_ROTATION:
                lockHeadingIfRotationStopped(input);
                ChassisSpeeds manualSpeeds = new ChassisSpeeds(
                        speedLimit.in(edu.wpi.first.units.Units.MetersPerSecond) * input.forward,
                        speedLimit.in(edu.wpi.first.units.Units.MetersPerSecond) * input.left,
                        Driving.kMaxRotationalRate.in(edu.wpi.first.units.Units.RadiansPerSecond) * input.rotation
                );
                swerve.setControl(
                        fieldCentricRequest
                                .withVelocityX(speedLimit.times(input.forward))
                                .withVelocityY(speedLimit.times(input.left))
                                .withRotationalRate(Driving.kMaxRotationalRate.times(input.rotation))
                );
                swerve.recordSetpointTelemetry(
                        new SwerveModuleState[]{ new SwerveModuleState(), new SwerveModuleState(),
                                new SwerveModuleState(), new SwerveModuleState() },
                        manualSpeeds
                );
                break;
            case DRIVING_WITH_LOCKED_HEADING:
                ChassisSpeeds lockedSpeeds = new ChassisSpeeds(
                        speedLimit.in(edu.wpi.first.units.Units.MetersPerSecond) * input.forward,
                        speedLimit.in(edu.wpi.first.units.Units.MetersPerSecond) * input.left,
                        0
                );
                swerve.setControl(
                        fieldCentricFacingAngleRequest
                                .withVelocityX(speedLimit.times(input.forward))
                                .withVelocityY(speedLimit.times(input.left))
                                .withTargetDirection(lockedHeading.get())
                );
                swerve.recordSetpointTelemetry(
                        new SwerveModuleState[]{ new SwerveModuleState(), new SwerveModuleState(),
                                new SwerveModuleState(), new SwerveModuleState() },
                        lockedSpeeds
                );
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // Default drive command: runs until interrupted
        return false;
    }
}