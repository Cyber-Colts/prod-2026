package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple linear-actuator-like wrapper around a PWM Servo with a position model
 * (mirrors the style of the provided Hood code but for a single servo).
 *
 * This keeps position tracking and allows small incremental moves (used with
 * POV/d-pad buttons).
 */
public class LinearActuator extends SubsystemBase {
    private static final Distance kServoLength = Millimeters.of(100);
    private static final LinearVelocity kMaxServoSpeed = Millimeters.of(20).per(Second);
    private static final double kMinPosition = 0.0;
    private static final double kMaxPosition = 1.0;
    private static final double kPositionTolerance = 0.005;

    private final Servo servo;

    private double currentPosition = 0.5;
    private double targetPosition = 0.5;
    private Time lastUpdateTime = Seconds.of(0);
    private final String name;

    public LinearActuator(int pwmChannel, String name) {
        this.name = name;
        servo = new Servo(pwmChannel);
        // Standard servo bounds; adjust if you have custom travel
        servo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    setPosition(currentPosition);
    // Publish this sendable to Shuffleboard under the "Linear Actuators" tab
    Shuffleboard.getTab("Linear Actuators").add(name, this);
    }

    /** Expects a position between 0.0 and 1.0 */
    public void setPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);
        servo.set(clampedPosition);
        targetPosition = clampedPosition;
    }

    /** Increment target position by delta (signed) */
    public void incrementPosition(double delta) {
        setPosition(targetPosition + delta);
    }

    public Command positionCommand(double position) {
        return runOnce(() -> setPosition(position))
            .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, currentPosition, kPositionTolerance);
    }

    private void updateCurrentPosition() {
        final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
        final Time elapsedTime = currentTime.minus(lastUpdateTime);
        lastUpdateTime = currentTime;

        if (isPositionWithinTolerance()) {
            currentPosition = targetPosition;
            return;
        }

        final Distance maxDistanceTraveled = kMaxServoSpeed.times(elapsedTime);
        final double maxPercentageTraveled = maxDistanceTraveled.div(kServoLength).in(Value);
        currentPosition = targetPosition > currentPosition
            ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
            : Math.max(targetPosition, currentPosition - maxPercentageTraveled);
    }

    @Override
    public void periodic() {
        updateCurrentPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Current Position", () -> currentPosition, null);
        builder.addDoubleProperty("Target Position", () -> targetPosition, value -> setPosition(value));
    }
}
