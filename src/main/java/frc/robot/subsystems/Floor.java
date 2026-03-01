package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Floor extends SubsystemBase {
    public enum Speed {
        STOP(0),
        FEED(0.83);

        private final double percentOutput;

        Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    private final SparkMax motor;
    private final RelativeEncoder encoder;

    public Floor() {
        motor = new SparkMax(Ports.kFloor, MotorType.kBrushless);
        encoder = motor.getEncoder();

        // Configure motor using new 2026 API
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)  // Supply current limit
            .secondaryCurrentLimit(40);  // Stator current limit

        // Apply configuration and persist to flash
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putData(this);
    }

    public void set(Speed speed) {
        motor.setVoltage(speed.voltage().in(Volts));
    }

    public Command feedCommand() {
        return startEnd(() -> set(Speed.FEED), () -> set(Speed.STOP));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("RPM", encoder::getVelocity, null);
        builder.addDoubleProperty("Output Current", motor::getOutputCurrent, null);
    }
}
