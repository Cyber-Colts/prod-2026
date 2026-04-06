package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;


public class Limelight extends SubsystemBase {
    private final String name;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;
    public static double linearStdDevBaseline = 0.08; // Meters
    public static double angularStdDevBaseline = 0.07; // Radians
    public static double angularStdDevMegatag2Factor =
            Double.POSITIVE_INFINITY;

    public Limelight(String name) {
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();
        
        // Configure the camera pose relative to the robot
        // This is critical for correct pose estimates when the camera isn't at the robot's origin
        LimelightHelpers.setCameraPose_RobotSpace(
            name,
            Constants.Limelight.CAMERA_FORWARD_OFFSET.in(Meter),
            Constants.Limelight.CAMERA_SIDE_OFFSET.in(Meter),
            Constants.Limelight.CAMERA_UP_OFFSET.in(Meter),
            Constants.Limelight.CAMERA_ROLL_DEGREES,
            Constants.Limelight.CAMERA_PITCH_DEGREES,
            Constants.Limelight.CAMERA_YAW_DEGREES
        );
    }

    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        // Get MegaTag estimates for the appropriate alliance
        var alliance = DriverStation.getAlliance();
        final PoseEstimate poseEstimate_MegaTag1;
        final PoseEstimate poseEstimate_MegaTag2;
        
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiRed(name);
            poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(name);
        } else {
            poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
            poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        }
        //Unhandled exception: java.lang.NullPointerException: Cannot read field "tagCount" because "poseEstimate_MegaTag1" is null
        if ( poseEstimate_MegaTag1 == null
                || poseEstimate_MegaTag2 == null
                || poseEstimate_MegaTag1.tagCount == 0
                || poseEstimate_MegaTag2.tagCount == 0
        ) {
            return Optional.empty();
        }

        // Combine the readings from MegaTag1 and MegaTag2:
        // 1. Use the more stable position from MegaTag2
        // 2. Use the rotation from MegaTag1 (with low confidence) to counteract gyro drift

        Pose2d pose1 = poseEstimate_MegaTag1.pose;
        Pose2d pose2 = poseEstimate_MegaTag2.pose;

        double error1 = pose1.getTranslation().getDistance(currentRobotPose.getTranslation());
        double error2 = pose2.getTranslation().getDistance(currentRobotPose.getTranslation());

        // Prefer MegaTag2 (more stable) unless MegaTag1 is significantly better
        // Only switch to MegaTag1 if it's more than 0.5m closer
        Pose2d selectedPose = (error1 < (error2 - 0.5)) ? pose1 : pose2;

        // poseEstimate_MegaTag2.pose = new Pose2d(
        //     poseEstimate_MegaTag2.pose.getTranslation(),
        //     poseEstimate_MegaTag1.pose.getRotation()
        // );
        poseEstimate_MegaTag2.pose = selectedPose;
        //TODO: remind test




        double stdDeviation = Math.pow(poseEstimate_MegaTag2.avgTagDist,2)/poseEstimate_MegaTag2.tagCount;
        double linearStdDev = stdDeviation * 0.1;
        double angularStdDev = stdDeviation * 0.2;
        // System.out.println(poseEstimate_MegaTag2.tagCount);
        //was 0.1 now 0.4 now
        //final Matrix<N3, N1> standardDeviations = VecBuilder.fill(0.8, 0.8, 10.0);
        final Matrix<N3, N1> standardDeviations = VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
        // System.out.println("angular" + angularStdDev);
        //System.out.println("linear" + linearStdDev);

        posePublisher.set(poseEstimate_MegaTag2.pose);

        return Optional.of(new Measurement(poseEstimate_MegaTag2, standardDeviations));
    }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
        }
    }
}