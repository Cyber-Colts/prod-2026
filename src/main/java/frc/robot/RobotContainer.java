// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Driving;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.*;
import frc.util.SwerveTelemetry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public static PathConstraints constraints = new PathConstraints(0.85, 0.85, 0.85, 0.85);

    private final Swerve swerve = Swerve.get();
    private final Intake intake = new Intake();
    private final Floor floor = new Floor();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Hanger hanger = new Hanger();
    private final Limelight limelight = new Limelight("limelight");
    private final NeoPixelStrip neoStrip = new NeoPixelStrip(Ports.kNeoPixel, 100);
    private final PlotLandmarks plotLandmarks = new PlotLandmarks();
    private SendableChooser<Command> autoChooser;

    private final CommandPS5Controller driver = new CommandPS5Controller(0);

    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX()
    );
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        LimelightHelpers.setupPortForwardingUSB(0);

        configureBindings();
        AutoRoutines autoRoutines = new AutoRoutines(
                swerve,
                intake,
                floor,
                feeder,
                shooter,
                hood,
                hanger,
                limelight
        );
        autoRoutines.configure();
        SwerveTelemetry swerveTelemetry = new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond));
        swerve.registerTelemetry(swerveTelemetry::telemeterize);
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("Start",
            AutoBuilder.pathfindToPose(Landmark.RIGHT_START.get(new Transform2d(Inches.of(0), Inches.of(0), Rotation2d.k180deg)), constraints, 0));

        NamedCommands.registerCommand("Tower",
            AutoBuilder.pathfindToPose(Landmark.TOWER.get(new Transform2d(Inches.of(Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 + 5 + 36), Inches.of(0), Rotation2d.k180deg)), constraints, 0));

        NamedCommands.registerCommand("Outpost",
            AutoBuilder.pathfindToPose(Landmark.OUTPOST.get(new Transform2d(Inches.of(Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 + 10), Inches.of(0), Rotation2d.k180deg)), constraints, 0));

        NamedCommands.registerCommand("Depot",
            AutoBuilder.pathfindToPose(Landmark.DEPOT.get(new Transform2d(Inches.of(0), Inches.of(0), Rotation2d.k180deg)), constraints, 0));

        NamedCommands.registerCommand("Hub",
            AutoBuilder.pathfindToPose(Landmark.HUB.get(new Transform2d(Inches.of(-24 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 - 2), Inches.of(0), Rotation2d.kZero)), constraints, 0));

        NamedCommands.registerCommand("RightBump",
            AutoBuilder.pathfindToPose(Landmark.RIGHT_BUMP.get(new Transform2d(Inches.of(-24 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 - 2), Inches.of(0), Rotation2d.fromDegrees(0))), constraints, 0));

        NamedCommands.registerCommand("LeftBump",
            AutoBuilder.pathfindToPose(Landmark.LEFT_BUMP.get(new Transform2d(Inches.of(-24 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 - 2), Inches.of(0), Rotation2d.fromDegrees(0))), constraints, 0));

        NamedCommands.registerCommand("RightTrench",
            AutoBuilder.pathfindToPose(Landmark.RIGHT_TRENCH.get(new Transform2d(Inches.of(-24 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 - 2), Inches.of(0), Rotation2d.fromDegrees(0))), constraints, 0));

        NamedCommands.registerCommand("LeftTrench",
            AutoBuilder.pathfindToPose(Landmark.LEFT_TRENCH.get(new Transform2d(Inches.of(-24 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 - 2), Inches.of(0), Rotation2d.fromDegrees(0))), constraints, 0));
    }
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * {@link edu.wpi.first.wpilibj2.command.button.CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        configureManualDriveBindings();
        limelight.setDefaultCommand(updateVisionCommand());

        // Idle the swerve while disabled so motors hold neutral mode instead of fighting
        RobotModeTriggers.disabled().whileTrue(
            swerve.applyRequest(SwerveRequest.Idle::new).ignoringDisable(true)
        );

        // LED state machine
        neoStrip.setBreathing(Color.kOrangeRed, 0.5, 0.05);
        RobotModeTriggers.teleop().onTrue(
            Commands.runOnce(() -> neoStrip.setDualBreathing(Color.kBlue, Color.kOrange, 2.0, 0.05))
        );
        RobotModeTriggers.autonomous().onTrue(
            Commands.runOnce(() -> neoStrip.setSolid(Color.kDarkMagenta))
        );
        RobotModeTriggers.disabled().onTrue(
            Commands.runOnce(() -> neoStrip.setBreathing(Color.kOrangeRed, 0.5, 0.05))
                .ignoringDisable(true)
        );

        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
            .onTrue(intake.homingCommand())
            .onTrue(hanger.homingCommand());

        driver.R2().whileTrue(subsystemCommands.aimAndShoot());
        driver.R1().whileTrue(subsystemCommands.shootManually());
        driver.L2().whileTrue(intake.intakeCommand());
        driver.L1().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));

        driver.povUp().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        driver.povDown().onTrue(hanger.positionCommand(Hanger.Position.HUNG));
        driver.povRight().whileTrue(feeder.spitCommand());
    }

    private void configureManualDriveBindings() {
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swerve, 
            () -> -driver.getLeftY(), 
            () -> -driver.getLeftX(), 
            () -> -driver.getRightX()
        );
        swerve.setDefaultCommand(manualDriveCommand);
        driver.cross().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.k180deg)));
        driver.circle().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCW_90deg)));
        driver.square().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCCW_90deg)));
        driver.triangle().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kZero)));
        driver.create().onTrue(Commands.runOnce(manualDriveCommand::seedFieldCentric));
    }

    private Command updateVisionCommand() {
        return limelight.run(() -> {
            final Pose2d currentRobotPose = swerve.getState().Pose;
            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                swerve.addVisionMeasurement(
                    m.poseEstimate.pose, 
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
        })
        .ignoringDisable(true);
    }
}
