package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
    public enum Mode { OFF, SOLID, RAINBOW, BREATHING, MOVING_RAINBOW }

    private AddressableLED led;
    private AddressableLEDBuffer buffer;
    private final int length;

    private boolean enabled = true;

    private Mode mode = Mode.OFF;
    private Color solidColor = new Color(0, 0, 0);

    // Rainbow state
    private int rainbowHue = 0;
    private double rainbowSpeed = 1.0; // hue increment per periodic call

    // Breathing state
    private double breathFrequencyHz = 0.5; // cycles per second
    private double breathMin = 0.1; // minimum brightness (0..1)

    // Shuffleboard entries
    private GenericEntry redEntry;
    private GenericEntry greenEntry;
    private GenericEntry blueEntry;
    private GenericEntry hueEntry;
    private GenericEntry rainbowSpeedEntry;
    private GenericEntry breathFreqEntry;
    private GenericEntry breathMinEntry;
    private final SendableChooser<Mode> modeChooser = new SendableChooser<>();

    /**
     * When true, the last programmatic call (setBreathing/setSolid/etc.)
     * takes priority and the Shuffleboard chooser is ignored.
     * Set to false to hand control back to Shuffleboard.
     */
    private boolean manualOverride = false;

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

        // Shuffleboard controls on the "NeoPixel" tab
        try {
            // Provide a safer chooser for modes instead of free-text
            modeChooser.setDefaultOption("OFF", Mode.OFF);
            modeChooser.addOption("SOLID", Mode.SOLID);
            modeChooser.addOption("RAINBOW", Mode.RAINBOW);
            modeChooser.addOption("MOVING_RAINBOW", Mode.MOVING_RAINBOW);
            modeChooser.addOption("BREATHING", Mode.BREATHING);
            Shuffleboard.getTab("NeoPixel").add("Mode Chooser", modeChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
            SimpleWidget wR = Shuffleboard.getTab("NeoPixel").add("Red", solidColor.red * 255.0).withWidget(BuiltInWidgets.kNumberSlider);
            redEntry = wR.getEntry();
            SimpleWidget wG = Shuffleboard.getTab("NeoPixel").add("Green", solidColor.green * 255.0).withWidget(BuiltInWidgets.kNumberSlider);
            greenEntry = wG.getEntry();
            SimpleWidget wB = Shuffleboard.getTab("NeoPixel").add("Blue", solidColor.blue * 255.0).withWidget(BuiltInWidgets.kNumberSlider);
            blueEntry = wB.getEntry();

            SimpleWidget wHue = Shuffleboard.getTab("NeoPixel").add("Hue", (double) rainbowHue).withWidget(BuiltInWidgets.kNumberSlider);
            hueEntry = wHue.getEntry();

            SimpleWidget wRainbowSpeed = Shuffleboard.getTab("NeoPixel").add("Rainbow Speed", rainbowSpeed).withWidget(BuiltInWidgets.kNumberSlider);
            rainbowSpeedEntry = wRainbowSpeed.getEntry();

            SimpleWidget wBreathFreq = Shuffleboard.getTab("NeoPixel").add("Breath Freq (Hz)", breathFrequencyHz).withWidget(BuiltInWidgets.kNumberSlider);
            breathFreqEntry = wBreathFreq.getEntry();

            SimpleWidget wBreathMin = Shuffleboard.getTab("NeoPixel").add("Breath Min (0-1)", breathMin).withWidget(BuiltInWidgets.kNumberSlider);
            breathMinEntry = wBreathMin.getEntry();

            // Initialize entries
            redEntry.setDouble(solidColor.red * 255.0);
            greenEntry.setDouble(solidColor.green * 255.0);
            blueEntry.setDouble(solidColor.blue * 255.0);
            hueEntry.setDouble(rainbowHue);
            rainbowSpeedEntry.setDouble(rainbowSpeed);
            breathFreqEntry.setDouble(breathFrequencyHz);
            breathMinEntry.setDouble(breathMin);
        } catch (Exception ex) {
            // If Shuffleboard isn't available at construction time, it's safe to ignore; periodic will still work.
        }
    }

    public void setSolid(Color color) {
        solidColor = color;
        mode = Mode.SOLID;
        manualOverride = true;
        if (enabled) {
            updateBufferSolid();
        }
    }

    public void setOff() {
        mode = Mode.OFF;
        manualOverride = true;
        if (enabled) {
            updateBufferOff();
        }
    }

    public void setRainbow() {
        mode = Mode.RAINBOW;
        manualOverride = true;
    }

    public void setMovingRainbow() {
        mode = Mode.MOVING_RAINBOW;
        manualOverride = true;
    }

    /** Breathe a specific color at the given frequency and minimum brightness. */
    public void setBreathing(Color color, double freqHz, double minBrightness) {
        solidColor = color;
        breathFrequencyHz = freqHz;
        breathMin = Math.max(0.0, Math.min(1.0, minBrightness));
        mode = Mode.BREATHING;
        manualOverride = true;
    }

    /** Breathe the current solidColor at the given frequency and minimum brightness. */
    public void setBreathing(double freqHz, double minBrightness) {
        breathFrequencyHz = freqHz;
        breathMin = Math.max(0.0, Math.min(1.0, minBrightness));
        mode = Mode.BREATHING;
        manualOverride = true;
    }

    /** Hand control back to the Shuffleboard mode chooser. */
    public void clearOverride() {
        manualOverride = false;
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

    private void updateBufferRainbow() {
        // Create a simple moving rainbow by setting hue per LED
        if (!enabled) {
            return;
        }
        for (int i = 0; i < length; i++) {
            int hue = (rainbowHue + (i * 180 / Math.max(1, length))) % 180; // 0-179 typical
            // Convert hue to RGB using HSB -> Color
            float h = hue / 180.0f; // 0-1
            int rgb = java.awt.Color.HSBtoRGB(h, 1.0f, 1.0f);
            int r = (rgb >> 16) & 0xFF;
            int g = (rgb >> 8) & 0xFF;
            int b = rgb & 0xFF;
            buffer.setLED(i, new Color(r / 255.0, g / 255.0, b / 255.0));
        }
        led.setData(buffer);
        rainbowHue = (rainbowHue + (int) Math.max(1, Math.round(rainbowSpeed))) % 180;
    }

    private void updateBufferMovingRainbow() {
        // Similar to updateBufferRainbow but uses fractional speed for smoother movement
        if (!enabled) {
            return;
        }
        for (int i = 0; i < length; i++) {
            int hue = (rainbowHue + (i * 180 / Math.max(1, length))) % 180;
            float h = hue / 180.0f;
            int rgb = java.awt.Color.HSBtoRGB(h, 1.0f, 1.0f);
            int r = (rgb >> 16) & 0xFF;
            int g = (rgb >> 8) & 0xFF;
            int b = rgb & 0xFF;
            buffer.setLED(i, new Color(r / 255.0, g / 255.0, b / 255.0));
        }
        led.setData(buffer);
        // advance hue by rainbowSpeed (can be fractional)
        rainbowHue = (int) ((rainbowHue + rainbowSpeed) % 180);
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

    @Override
    public void periodic() {
        // Update according to mode
        // If a programmatic call set manualOverride, skip the chooser entirely.
        // Otherwise let Shuffleboard chooser drive the mode.
        try {
            if (!manualOverride) {
                Mode chosen = modeChooser.getSelected();
                if (chosen != null && chosen != mode) {
                    mode = chosen;
                    if (mode == Mode.SOLID) {
                        updateBufferSolid();
                    } else if (mode == Mode.OFF) {
                        updateBufferOff();
                    }
                }
            }
        } catch (Exception e) {
            // ignore chooser failures
        }

        // If in solid mode, update color from Shuffleboard sliders
        try {
            if (mode == Mode.SOLID && redEntry != null && greenEntry != null && blueEntry != null) {
                double r = Math.max(0.0, Math.min(255.0, redEntry.getDouble(solidColor.red * 255.0)));
                double g = Math.max(0.0, Math.min(255.0, greenEntry.getDouble(solidColor.green * 255.0)));
                double b = Math.max(0.0, Math.min(255.0, blueEntry.getDouble(solidColor.blue * 255.0)));
                Color newColor = new Color(r / 255.0, g / 255.0, b / 255.0);
                if (newColor.red != solidColor.red || newColor.green != solidColor.green || newColor.blue != solidColor.blue) {
                    solidColor = newColor;
                    updateBufferSolid();
                }
            }
        } catch (Exception e) {
            // ignore
        }

        // If in rainbow mode or moving rainbow, optionally read hue offset and speed
        try {
            if ((mode == Mode.RAINBOW || mode == Mode.MOVING_RAINBOW) && hueEntry != null) {
                int h = (int) hueEntry.getDouble(rainbowHue);
                rainbowHue = Math.max(0, Math.min(179, h));
            }
            if (rainbowSpeedEntry != null) {
                double s = rainbowSpeedEntry.getDouble(rainbowSpeed);
                rainbowSpeed = Math.max(0.0, s);
            }
        } catch (Exception e) {
            // ignore
        }

        // If in breathing mode, optionally read frequency/min from Shuffleboard
        try {
            if (breathFreqEntry != null) {
                double f = breathFreqEntry.getDouble(breathFrequencyHz);
                breathFrequencyHz = Math.max(0.0, f);
            }
            if (breathMinEntry != null) {
                double bm = breathMinEntry.getDouble(breathMin);
                breathMin = Math.max(0.0, Math.min(1.0, bm));
            }
        } catch (Exception e) {
            // ignore
        }

        switch (mode) {
            case OFF:
                // nothing to do; already off
                break;
            case SOLID:
                // ensure buffer has solid color (cheap)
                updateBufferSolid();
                break;
            case RAINBOW:
                updateBufferRainbow();
                break;
            case MOVING_RAINBOW:
                updateBufferMovingRainbow();
                break;
            case BREATHING:
                updateBufferBreathing();
                break;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Mode", () -> mode.name(), null);
    builder.addDoubleProperty("Hue", () -> (double) rainbowHue, val -> rainbowHue = (int) Math.max(0, Math.min(179, (int) val)));
        builder.addStringProperty("Color", () -> String.format("#%02X%02X%02X", (int)(solidColor.red*255), (int)(solidColor.green*255), (int)(solidColor.blue*255)), null);
    }
}
