package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple Addressable LED (NeoPixel) strip wrapper.
 *
 * Note: Adjust `length` to match your physical strip. This uses a PWM port
 * on the RoboRIO (e.g., PWM 2) for the data line.
 */
public class NeoPixelStrip extends SubsystemBase {
    public enum Mode { OFF, SOLID, BREATHING, DUAL_BREATHING }

    private AddressableLED led;
    private AddressableLEDBuffer buffer;
    private final int length;

    private boolean enabled = true;

    private Mode mode = Mode.OFF;
    private Color solidColor = new Color(0, 0, 0);
    private Color secondaryColor = new Color(0, 0, 0);

    // Breathing state
    private double breathFrequencyHz = 0.5; // cycles per second
    private double breathMin = 0.1; // minimum brightness (0..1)

    public NeoPixelStrip(int pwmPort, int length) {
        this.length = Math.max(1, length);
        try {
            led = new AddressableLED(pwmPort);
            buffer = new AddressableLEDBuffer(this.length);
            led.setLength(this.length);
            led.setData(buffer);
            led.start();
            setOff();
        } catch (Throwable t) {
            // HAL or JNI not available (e.g., running in simulator or hardware not present).
            // Disable LED functionality but keep the robot running.
            led = null;
            buffer = null;
            enabled = false;
            DriverStation.reportWarning("AddressableLED unavailable; NeoPixelStrip disabled: " + t.getMessage(), t.getStackTrace());
        }
    }

    public void setSolid(Color color) {
        solidColor = color;
        mode = Mode.SOLID;
        if (enabled) {
            updateBufferSolid();
        }
    }

    public void setOff() {
        mode = Mode.OFF;
        if (enabled) {
            updateBufferOff();
        }
    }

    public void setDualBreathing(Color c1, Color c2, double freqHz, double minBrightness) {
        solidColor = c1;
        secondaryColor = c2;
        breathFrequencyHz = freqHz;
        breathMin = Math.max(0.0, Math.min(1.0, minBrightness));
        mode = Mode.DUAL_BREATHING;
    }

    /** Breathe a specific color at the given frequency and minimum brightness. */
    public void setBreathing(Color color, double freqHz, double minBrightness) {
        solidColor = color;
        breathFrequencyHz = freqHz;
        breathMin = Math.max(0.0, Math.min(1.0, minBrightness));
        mode = Mode.BREATHING;
    }

    /** Breathe the current solidColor at the given frequency and minimum brightness. */
    public void setBreathing(double freqHz, double minBrightness) {
        breathFrequencyHz = freqHz;
        breathMin = Math.max(0.0, Math.min(1.0, minBrightness));
        mode = Mode.BREATHING;
    }

    private void updateBufferSolid() {
        if (!enabled) {
            return;
        }
        for (int i = 0; i < length; i++) {
            buffer.setLED(i, solidColor);
        }
        led.setData(buffer);
    }

    private void updateBufferOff() {
        if (!enabled) {
            return;
        }
        for (int i = 0; i < length; i++) {
            buffer.setLED(i, new Color(0.0, 0.0, 0.0));
        }
        led.setData(buffer);
    }

    private void updateBufferBreathing() {
        if (!enabled) {
            return;
        }
        double t = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double phase = 2.0 * Math.PI * breathFrequencyHz * t;
        double sine = 0.5 * (1.0 + Math.sin(phase)); // 0..1
        double brightness = breathMin + (1.0 - breathMin) * sine; // breathMin..1.0
        for (int i = 0; i < length; i++) {
            buffer.setLED(i, new Color(solidColor.red * brightness, solidColor.green * brightness, solidColor.blue * brightness));
        }
        led.setData(buffer);
    }

    private void updateBufferDualBreathing() {
        if (!enabled) {
            return;
        }
        double t = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double phase = 2.0 * Math.PI * breathFrequencyHz * t;
        double sine = 0.5 * (1.0 + Math.sin(phase)); // 0..1
        double brightness = breathMin + (1.0 - breathMin) * sine; // breathMin..1.0
        for (int i = 0; i < length; i++) {
            if (i < length / 2) {
                buffer.setLED(i, new Color(solidColor.red * brightness, solidColor.green * brightness, solidColor.blue * brightness));
            } else {
                buffer.setLED(i, new Color(secondaryColor.red * brightness, secondaryColor.green * brightness, secondaryColor.blue * brightness));
            }
        }
        led.setData(buffer);
    }

    @Override
    public void periodic() {
        switch (mode) {
            case OFF:
                break;
            case SOLID:
                // ensure buffer has solid color (cheap)
                updateBufferSolid();
                break;
            case BREATHING:
                updateBufferBreathing();
                break;
            case DUAL_BREATHING:
                updateBufferDualBreathing();
                break;
        }
    }
}
