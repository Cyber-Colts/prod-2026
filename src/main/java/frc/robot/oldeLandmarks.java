package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/*
 * The origin for the field is in the top right
 * The x axis goes through both hubs
 * Rotation is 0 when its facing the red alliance
 * In here, all the landmarks' rotations are set to face the opposite alliance
 */

public class oldeLandmarks {
    public Distance fieldWidth = Inches.of(651.22);
    public Distance fieldHeight = Inches.of(317.69);

    //public static Pose2d getPose() {

    // }
    // Center of the hub

   /*  public static Translation2d hubPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Translation2d(Inches.of(182.105), Inches.of(158.845));
        }
        return new Translation2d(Inches.of(469.115), Inches.of(158.845));
    }*/

    public static Pose2d startPosition() {
        Pose2d startPos = new Pose2d(Inches.of(182.105-44), Inches.of(20), Rotation2d.fromDegrees(180));
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return startPos;
        }
        return startPos;
    }


    public static Pose2d hubPosition() { //
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Pose2d(Inches.of(182.105), Inches.of(158.845), Rotation2d.fromDegrees(0));
        }
        return new Pose2d(Inches.of(469.115), Inches.of(158.845), Rotation2d.fromDegrees(180));
    }

    // Position is either the center of the rungs part of the tower or its the very forward part of the rungs, centered.
    // Outpost and tower - rotation is facing opposite alliance
    public static Pose2d towerPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            // was 49.57 for x before
            return new Pose2d(Inches.of(68.57), Inches.of(147.47), Rotation2d.fromDegrees(180));
        }
        return new Pose2d(Inches.of(601.65), Inches.of(253.22), Rotation2d.fromDegrees(0));
        // The distance from the wall to the tower ma ybe incorrect because its from the game manual
    }


    // position is where the center april tag is
    public static Pose2d outpostPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            //was 0.3 for x before
            return new Pose2d(Inches.of(32.30), Inches.of(26.22), Rotation2d.fromDegrees(180));//0 degrees before
        }
        return new Pose2d(Inches.of(650.92), Inches.of(291.47), Rotation2d.fromDegrees(0)); // 180 degrees before
    }

    // Left/right is based from the perspective of you looking from the alliance station
    // Position is the center of the bump on both sides
    // Rotation is facing your alliance side

    public static Pose2d rightBumpPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Pose2d(Inches.of(465.119), Inches.of(98.845), Rotation2d.fromDegrees(0));
        }
        return new Pose2d(Inches.of(182.105), Inches.of(218.845), Rotation2d.fromDegrees(180));
    }

    public static Pose2d leftBumpPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Pose2d(Inches.of(465.119), Inches.of(218.844), Rotation2d.fromDegrees(0));
        }
        return new Pose2d(Inches.of(182.105), Inches.of(98.844), Rotation2d.fromDegrees(180));
    }

    public static Pose2d rightTrenchPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Pose2d(Inches.of(182.105), Inches.of(25.375), Rotation2d.fromDegrees(0));
        }
        return new Pose2d(Inches.of(465.119), Inches.of(292.315), Rotation2d.fromDegrees(180));
    }

    public static Pose2d leftTrenchPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Pose2d(Inches.of(182.105), Inches.of(292.315), Rotation2d.fromDegrees(0));
        }
        return new Pose2d(Inches.of(465.119), Inches.of(25.375), Rotation2d.fromDegrees(180));
    }


}