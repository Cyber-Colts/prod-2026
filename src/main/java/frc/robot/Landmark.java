package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

public enum Landmark {
    // Note: Bumper width is 35 inches
    // In the 2026 game, the origin is the bottom right corner from the blue alliance's perspective.
    OUTPOST(new Pose2d(Inches.of(0.32), Inches.of(26.22), Rotation2d.fromDegrees(0))),
    DEPOT(new Pose2d(Inches.of(0.32), Inches.of(234.32), Rotation2d.fromDegrees(0))),
    TOWER(new Pose2d(Inches.of(0.32), Inches.of(147.47), Rotation2d.fromDegrees(0))),
    HUB(new Pose2d(Inches.of(182.105), Inches.of(158.845), Rotation2d.fromDegrees(0))),
    RIGHT_BUMP(new Pose2d(Inches.of(182.105), Inches.of(98.845), Rotation2d.fromDegrees(0))),
    LEFT_BUMP(new Pose2d(Inches.of(182.105), Inches.of(218.845), Rotation2d.fromDegrees(0))),
    RIGHT_TRENCH(new Pose2d(Inches.of(182.105), Inches.of(25.375), Rotation2d.fromDegrees(0))),
    LEFT_TRENCH(new Pose2d(Inches.of(182.105), Inches.of(292.815), Rotation2d.fromDegrees(0))),

    // The robot's bumper is expected to overlap since the line's thickness will extend it a little towards the alliance side
    RIGHT_START(new Pose2d(Inches.of(158.34 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5), Inches.of(25.375), Rotation2d.fromDegrees(0))),
    LEFT_START(new Pose2d(Inches.of(158.34 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5), Inches.of(292.815), Rotation2d.fromDegrees(0))),
    MIDDLE_START(new Pose2d(Inches.of(158.34 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5), Inches.of(158.845), Rotation2d.fromDegrees(0)));

    private Pose2d bluePose;
    private Pose2d redPose;

    Landmark(Pose2d bluePose) {
        this.bluePose = bluePose;
    }

    public static Pose2d flipAlliance(Pose2d original) {
        return new Pose2d(Constants.FieldConstants.FIELD_WIDTH.minus(original.getMeasureX()), Constants.FieldConstants.FIELD_HEIGHT.minus(original.getMeasureY()), original.getRotation().rotateBy(Rotation2d.k180deg));
    }

    public Pose2d get() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            if (redPose == null)
                redPose = flipAlliance(bluePose);
            return redPose;
        }
        return bluePose;
    }

    public Pose2d get(Alliance alliance) {
        if (alliance == Alliance.Red) {
            if (redPose == null)
                redPose = flipAlliance(bluePose);
            return redPose;
        }
        return bluePose;
    }

    public Pose2d get(Transform2d adjustment) {
        return get().plus(adjustment);
    }

    public Pose2d get(Alliance alliance, Transform2d adjustment) {
        return get(alliance).plus(adjustment);
    }


    //  - Add a function that can return the nearest trench/bump
    public Pose2d getNearest(Pose2d landmarkUno, Pose2d landmarkDos) {
        Pose2d robotPose = Swerve.get().getState().Pose;
        double rightDistance =
                Math.pow(
                        Math.pow(
                                Math.abs(landmarkUno.getX()-robotPose.getX()), 2
                        ) +
                                Math.pow(
                                        Math.abs(landmarkUno.getY()-robotPose.getY()), 2
                                ),
                        0.5);
        double leftDistance =
                Math.pow(
                        Math.pow(
                                Math.abs(landmarkDos.getX()-robotPose.getX()), 2
                        ) +
                                Math.pow(
                                        Math.abs(landmarkDos.getY()-robotPose.getY()), 2
                                ),
                        0.5);
        return leftDistance > rightDistance ? landmarkUno : landmarkDos;
    }
    public Pose2d getNearestBump() {
        return getNearest(LEFT_BUMP.get(), RIGHT_BUMP.get());
    }

    public Pose2d getNearestTrench() {
        return getNearest(LEFT_TRENCH.get(), RIGHT_TRENCH.get());
    }
}