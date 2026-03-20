// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.Inches;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * This enum defines the runtime mode used by AdvantageKit. The mode is always "real" when running
     * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
     * (log replay from a file).
     */
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static class Driving {
        public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1);
        public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005); // 0.5% of max rotational speed
    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }
    public static class FieldConstants {
        public static final Distance FIELD_WIDTH = Inches.of(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded).getFieldWidth());
        public static final Distance FIELD_HEIGHT = Inches.of(317.69);
    }

    public static class RobotDimensions {
        public static final Distance BUMPER_WIDTH = Inches.of(35); //
        public static final Distance ROBOT_WIDTH = Inches.of(25.625);
        public static final Distance ROBOT_HEIGHT = Inches.of(18.5); //
    }

    public static class Limelight {
        // Camera position relative to robot center (forward, side, up) in inches
        public static final double CAMERA_FORWARD_OFFSET_INCHES = 27.027;
        public static final double CAMERA_SIDE_OFFSET_INCHES = 0.0;
        public static final double CAMERA_UP_OFFSET_INCHES = 31.066;
        
        // Camera rotation relative to robot (roll, pitch, yaw) in degrees
        // Adjust these based on how your Limelight camera is physically mounted
        // If the camera is rotated 90 degrees clockwise (right), set YAW to 90
        public static final double CAMERA_ROLL_DEGREES = 15.0;
        public static final double CAMERA_PITCH_DEGREES = 0.0;
        public static final double CAMERA_YAW_DEGREES = 0; // Adjust this if camera is rotated
    }
}
