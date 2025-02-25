// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class AutoConstants {
    public static final double AUTO_MAX_VELOCITY_METERS_PER_SECOND = 4.0; // TODO update value
    public static final double AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 5.0; // TODO update value
    public static final double AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC = 2 * Math.PI; // TODO update value
    public static final double AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 2 * Math.PI; // TODO update value

    public static final double X_DRIVE_P = 2.05;
    public static final double X_DRIVE_I = 0;
    public static final double X_DRIVE_D = 0.1;

    public static final double Y_DRIVE_P = 2.05;
    public static final double Y_DRIVE_I = 0;
    public static final double Y_DRIVE_D = 0.1;

    public static final double THETA_P = 10.0;
    public static final double THETA_I = 0;
    public static final double THETA_D = 0;
  }

  public static class AlignConstants {
    public static final double ALIGN_P = 4;
  }

  public static class TrajectoryConstants {
  
  public static final Pose2d R_HP_LEFT_CENTER = new Pose2d (new Translation2d(Inches.of(646.79), Inches.of(40.362)), new Rotation2d(Degrees.of(306)));
  public static final Pose2d R_HP_RIGHT_CENTER = new Pose2d (new Translation2d(Inches.of(646.79), Inches.of(276.638)), new Rotation2d(Degrees.of(414)));
  public static final Pose2d B_HP_RIGHT_CENTER = new Pose2d (new Translation2d(Inches.of(44.09), Inches.of(40.362)), new Rotation2d(Degrees.of(234)));
  public static final Pose2d B_HP_LEFT_CENTER = new Pose2d (new Translation2d(Inches.of(44.09), Inches.of(276.638)), new Rotation2d(Degrees.of(486)));

  public static final Pose2d R_HP_LEFT_OUT = new Pose2d (new Translation2d(Inches.of(633.85), Inches.of(30.962)), new Rotation2d(Degrees.of(306)));
  public static final Pose2d R_HP_RIGHT_OUT = new Pose2d (new Translation2d(Inches.of(633.85), Inches.of(286.038)), new Rotation2d(Degrees.of(414)));
  public static final Pose2d B_HP_RIGHT_OUT = new Pose2d (new Translation2d(Inches.of(57.03), Inches.of(30.962)), new Rotation2d(Degrees.of(234)));
  public static final Pose2d B_HP_LEFT_OUT = new Pose2d (new Translation2d(Inches.of(57.03), Inches.of(286.038)), new Rotation2d(Degrees.of(486)));

  public static final Pose2d R_HP_LEFT_IN = new Pose2d (new Translation2d(Inches.of(659.73), Inches.of(49.762)), new Rotation2d(Degrees.of(306)));
  public static final Pose2d R_HP_RIGHT_IN = new Pose2d (new Translation2d(Inches.of(659.73), Inches.of(267.238)), new Rotation2d(Degrees.of(414)));
  public static final Pose2d B_HP_RIGHT_IN = new Pose2d (new Translation2d(Inches.of(31.15), Inches.of(49.762)), new Rotation2d(Degrees.of(234)));
  public static final Pose2d B_HP_LEFT_IN = new Pose2d (new Translation2d(Inches.of(31.15), Inches.of(267.238)), new Rotation2d(Degrees.of(486)));
    
  public static final Pose2d RED_A = new Pose2d (new Translation2d(Inches.of(564.87), Inches.of(153)), new Rotation2d(Degrees.of(0)));
  public static final Pose2d RED_B = new Pose2d (new Translation2d(Inches.of(564.87), Inches.of(166)), new Rotation2d(Degrees.of(0)));
  public static final Pose2d RED_C = new Pose2d (new Translation2d(Inches.of(544.253), Inches.of(199.668)), new Rotation2d(Degrees.of(60)));
  public static final Pose2d RED_D = new Pose2d (new Translation2d(Inches.of(532.995), Inches.of(206.168)), new Rotation2d(Degrees.of(60)));
  public static final Pose2d RED_E = new Pose2d (new Translation2d(Inches.of(493.533), Inches.of(205.168)), new Rotation2d(Degrees.of(120)));
  public static final Pose2d RED_F = new Pose2d (new Translation2d(Inches.of(482.275), Inches.of(198.668)), new Rotation2d(Degrees.of(120)));
  public static final Pose2d RED_G = new Pose2d (new Translation2d(Inches.of(463.39), Inches.of(164)), new Rotation2d(Degrees.of(180)));
  public static final Pose2d RED_H = new Pose2d (new Translation2d(Inches.of(463.39), Inches.of(151)), new Rotation2d(Degrees.of(180)));
  public static final Pose2d RED_I = new Pose2d (new Translation2d(Inches.of(484.007), Inches.of(117.332)), new Rotation2d(Degrees.of(240)));
  public static final Pose2d RED_J = new Pose2d (new Translation2d(Inches.of(495.265), Inches.of(110.832)), new Rotation2d(Degrees.of(240)));
  public static final Pose2d RED_K = new Pose2d (new Translation2d(Inches.of(534.727), Inches.of(111.832)), new Rotation2d(Degrees.of(300)));
  public static final Pose2d RED_L = new Pose2d (new Translation2d(Inches.of(545.985), Inches.of(118.332)), new Rotation2d(Degrees.of(300)));

  public static final Pose2d BLUE_A = new Pose2d (new Translation2d(Inches.of(126), Inches.of(164)), new Rotation2d(Degrees.of(180)));
  public static final Pose2d BLUE_B = new Pose2d (new Translation2d(Inches.of(126), Inches.of(151)), new Rotation2d(Degrees.of(180)));
  public static final Pose2d BLUE_C = new Pose2d (new Translation2d(Inches.of(146.627), Inches.of(117.332)), new Rotation2d(Degrees.of(240)));
  public static final Pose2d BLUE_D = new Pose2d (new Translation2d(Inches.of(157.885), Inches.of(110.832)), new Rotation2d(Degrees.of(240)));
  public static final Pose2d BLUE_E = new Pose2d (new Translation2d(Inches.of(197.337), Inches.of(111.832)), new Rotation2d(Degrees.of(300)));
  public static final Pose2d BLUE_F = new Pose2d (new Translation2d(Inches.of(208.595), Inches.of(118.332)), new Rotation2d(Degrees.of(300)));
  public static final Pose2d BLUE_G = new Pose2d (new Translation2d(Inches.of(227.49), Inches.of(153)), new Rotation2d(Degrees.of(0)));
  public static final Pose2d BLUE_H = new Pose2d (new Translation2d(Inches.of(227.49), Inches.of(166)), new Rotation2d(Degrees.of(0)));
  public static final Pose2d BLUE_I = new Pose2d (new Translation2d(Inches.of(206.863), Inches.of(199.668)), new Rotation2d(Degrees.of(60)));
  public static final Pose2d BLUE_J = new Pose2d (new Translation2d(Inches.of(195.605), Inches.of(206.168)), new Rotation2d(Degrees.of(60)));
  public static final Pose2d BLUE_K = new Pose2d (new Translation2d(Inches.of(156.153), Inches.of(205.168)), new Rotation2d(Degrees.of(120)));
  public static final Pose2d BLUE_L = new Pose2d (new Translation2d(Inches.of(144.895), Inches.of(198.668)), new Rotation2d(Degrees.of(120)));
      
    public static final Pose2d RED_RB_START = new Pose2d (new Translation2d(Inches.of(400), Inches.of(75.4)), new Rotation2d(Degrees.of(180)));
    public static final Pose2d RED_BB_START = new Pose2d (new Translation2d(Inches.of(400), Inches.of(241.6)), new Rotation2d(Degrees.of(180)));
    public static final Pose2d BLUE_RB_START = new Pose2d (new Translation2d(Inches.of(290), Inches.of(75.4)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d BLUE_BB_START = new Pose2d (new Translation2d(Inches.of(290), Inches.of(241.6)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d RED_SINGLE_START = new Pose2d (new Translation2d(Inches.of(400), Inches.of(152)), new Rotation2d(Degrees.of(180)));
    public static final Pose2d BLUE_SINGLE_START = new Pose2d (new Translation2d(Inches.of(290), Inches.of(152)), new Rotation2d(Degrees.of(180)));

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final double DRIVE_SLEW_RATE = 10.0;
  }

  public static class VisionConstants {
    public static final double FL_OFFSETX_MET = 0;
    public static final double FL_OFFSETY_MET = 0;
    public static final double FL_OFFSETZ_MET = 0;
    public static final double FL_PITCH = 0;
    public static final double FL_YAW = 0;

    public static final double FR_OFFSETX_MET = 0;
    public static final double FR_OFFSETY_MET = 0;
    public static final double FR_OFFSETZ_MET = 0;
    public static final double FR_PITCH = 0;
    public static final double FR_YAW = 0;

    public static final double BACK_OFFSETX_MET = 0;
    public static final double BACK_OFFSETY_MET = 0;
    public static final double BACK_OFFSETZ_MET = 0;
    public static final double BACK_PITCH = 0;
    public static final double BACK_YAW = 0;
  }

  public static class ReefAlignConstants {
    public static final double THETA_P = 0;
  }
}
