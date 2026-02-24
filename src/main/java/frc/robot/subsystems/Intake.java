package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Intake extends SubsystemBase {
    public enum Speed {
        STOP(0),
        INTAKE(0.8);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    public enum Position {
        HOMED(110),
        STOWED(100),
        INTAKE(-4),
        AGITATE(20);

        private final double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }

        public Angle angle() {
            return Degrees.of(degrees);
        }
    }

    private static final double kPivotReduction = 50.0;
    private static final double kNEOFreeSpeed = 5880.0; // RPM
    private static final AngularVelocity kMaxPivotSpeed = RPM.of(kNEOFreeSpeed).div(kPivotReduction);
    private static final Angle kPositionTolerance = Degrees.of(5);

    private final SparkMax pivotMotor;
    private final RelativeEncoder pivotEncoder;
    private final TalonFX rollerMotor;
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);

    private double targetPivotPosition = 0.0;

    private boolean isHomed = false;

    public Intake() {
        pivotMotor = new SparkMax(Ports.kIntakePivot, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        rollerMotor = new TalonFX(Ports.kIntakeRollers, Ports.kCANivoreCANBus);
        configurePivotMotor();
        configureRollerMotor();
        SmartDashboard.putData(this);
    }

    private void configurePivotMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);  // Supply current limit in Amps

        // Configure the encoder
        config.encoder
            .positionConversionFactor(360.0 / kPivotReduction)  // Convert to degrees
            .velocityConversionFactor(360.0 / kPivotReduction / 60.0);  // Convert to degrees per second

        // Configure PID controller for position control
        config.closedLoop
            .p(0.3)
            .i(0)
            .d(0);

        // Configure feedforward (SparkFlex style, match Kraken kV usage)
        config.closedLoop.feedForward
            .kS(0)
            .kV(12.0 / kMaxPivotSpeed.in(RotationsPerSecond))  // 12 volts when requesting max RPS
            .kA(0)
            .kG(0)
            .kCos(0)
            .kCosRatio(1.0);

        // Apply configuration and persist to flash
        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureRollerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            );
        rollerMotor.getConfigurator().apply(config);
    }

    private boolean isPositionWithinTolerance() {
        final double currentPosition = pivotEncoder.getPosition();
        return Math.abs(currentPosition - targetPivotPosition) <= kPositionTolerance.in(Degrees);
    }

    private void setPivotPercentOutput(double percentOutput) {
        pivotMotor.set(percentOutput);
    }

    @SuppressWarnings("removal")
    public void set(Position position) {
        targetPivotPosition = position.angle().in(Degrees);
        pivotMotor.getClosedLoopController().setReference(
            targetPivotPosition,
            com.revrobotics.spark.SparkBase.ControlType.kPosition
        );
    }

    public void set(Speed speed) {
        rollerMotor.setControl(
            rollerVoltageRequest
                .withOutput(speed.voltage())
        );
    }

    public Command intakeCommand() {
        return startEnd(
            () -> {
                set(Position.INTAKE);
                set(Speed.INTAKE);
            },
            () -> set(Speed.STOP)
        );
    }

    public Command agitateCommand() {
        return runOnce(() -> set(Speed.INTAKE))
            .andThen(
                Commands.sequence(
                    runOnce(() -> set(Position.AGITATE)),
                    Commands.waitUntil(this::isPositionWithinTolerance),
                    runOnce(() -> set(Position.INTAKE)),
                    Commands.waitUntil(this::isPositionWithinTolerance)
                )
                .repeatedly()
            )
            .handleInterrupt(() -> {
                set(Position.INTAKE);
                set(Speed.STOP);
            });
    }

    public Command homingCommand() {
        return Commands.sequence(
            runOnce(() -> setPivotPercentOutput(0.1)),
            Commands.waitUntil(() -> pivotMotor.getOutputCurrent() > 6),
            runOnce(() -> {
                pivotEncoder.setPosition(0);
                isHomed = true;
                set(Position.STOWED);
            })
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Angle (degrees)", pivotEncoder::getPosition, null);
        builder.addDoubleProperty("RPM", () -> rollerMotor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty("Pivot Output Current", pivotMotor::getOutputCurrent, null);
        builder.addDoubleProperty("Roller Supply Current", () -> rollerMotor.getSupplyCurrent().getValue().in(Amps), null);
    }
}
