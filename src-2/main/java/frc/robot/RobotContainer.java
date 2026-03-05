// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.NeoPixelStrip;
import frc.robot.subsystems.LinearActuator;


import frc.robot.commands.AlignCommand;
import frc.robot.commands.TimedAlignCommand;
import frc.robot.commands.TestDriveCommand;

public class RobotContainer {
    //private final SendableChooser<Command> autoChooser;
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    // Two simple PWM-driven linear actuators on channels 0 and 1
    private final LinearActuator actuator0 = new LinearActuator(0, "Linear Actuator 0");
    private final LinearActuator actuator1 = new LinearActuator(1, "Linear Actuator 1");
    private final NeoPixelStrip neoStrip = new NeoPixelStrip(6, 100);
    

    private final Command alignCommand = new AlignCommand(m_visionSubsystem, drivetrain, 0.35, 0);
    private final Command timedAlignCommand = new TimedAlignCommand(m_visionSubsystem, drivetrain, 0.35, 0, 1.5);
    
    private final Command adjustLeft = new TestDriveCommand(drivetrain, -0.75, 0.5);
    private final Command adjustRight = new TestDriveCommand(drivetrain, 0.75,0.5);

    public RobotContainer() {
        NamedCommands.registerCommand("Adjust Left", adjustLeft);
        NamedCommands.registerCommand("Adjust Right", adjustRight);
        NamedCommands.registerCommand("Align", alignCommand);

        NamedCommands.registerCommand("TimedAlign", timedAlignCommand);
        configureBindings();
        // Default: breathe blue at 0.5 Hz
        neoStrip.setBreathing(edu.wpi.first.wpilibj.util.Color.kBlue, 0.5, 0.05);
        //autoChooser = AutoBuilder.buildAutoChooser();
        //SmartDashboard.putData("Auto Chooser", autoChooser);
    
    }

   private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                !alignCommand.isScheduled()
                ? drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                : drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // LED mode based on robot state
        RobotModeTriggers.teleop().onTrue(
            Commands.runOnce(() -> neoStrip.setBreathing(edu.wpi.first.wpilibj.util.Color.kAquamarine, 2.0, 0.05))
        );
        RobotModeTriggers.disabled().onTrue(
            Commands.runOnce(() -> neoStrip.setBreathing(edu.wpi.first.wpilibj.util.Color.kBlue, 0.5, 0.05))
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        
        joystick.rightTrigger().whileTrue(alignCommand);

        drivetrain.registerTelemetry(logger::telemeterize);

        // POV (D-pad) controls: move both actuators together.
        // POV Up/Down will move actuator0 and actuator1 simultaneously.
        final double deltaPerCycle = 0.01; // small incremental step per scheduler cycle
        joystick.povUp().whileTrue(Commands.run(() -> {
            actuator0.incrementPosition(deltaPerCycle);
            actuator1.incrementPosition(deltaPerCycle);
        }, actuator0, actuator1));
        joystick.povDown().whileTrue(Commands.run(() -> {
            actuator0.incrementPosition(-deltaPerCycle);
            actuator1.incrementPosition(-deltaPerCycle);
        }, actuator0, actuator1));
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
