// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.Map;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {


  public static class OperatorConstants {

    //SET BOTH TO 0 for SINGLE CONTROLER COMMAND

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 0;
    }

   public static class VisionConstants {
      public static final String LIMELIGHTA_NAME = "limelight.local";
      public static final double LIMELIGHTA_LENS_HEIGHT = 11.5; //Find later
      public static final double LIMELIGHTA_ANGLE = 50;//Find later
      }
    

  }