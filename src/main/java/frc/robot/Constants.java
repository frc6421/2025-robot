// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class AutoConstants {
    public static final double AUTO_MAX_VELOCITY_METERS_PER_SECOND = 4.0; // TODO update value
    public static final double AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 5.0; // TODO update value
    public static final double AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC = 2 * Math.PI; // TODO update value
    public static final double AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 2 * Math.PI; // TODO update value

    public static final double X_DRIVE_P = 2.32;
    public static final double X_DRIVE_I = 0;
    public static final double X_DRIVE_D = 0.1;

    public static final double Y_DRIVE_P = 2.32;
    public static final double Y_DRIVE_I = 0;
    public static final double Y_DRIVE_D = 0.1;

    public static final double THETA_P = 5.0;
    public static final double THETA_I = 0;
    public static final double THETA_D = 0;
  }

  public static class TrajectoryConstants {
  
    public static final Pose2d R_HP_LEFT_CENTER = new Pose2d (new Translation2d(Inches.of(647.378), Inches.of(39.553)), new Rotation2d(Degrees.of(306)));
    public static final Pose2d R_HP_RIGHT_CENTER = new Pose2d (new Translation2d(Inches.of(647.378), Inches.of(277.447)), new Rotation2d(Degrees.of(414)));
    public static final Pose2d B_HP_RIGHT_CENTER = new Pose2d (new Translation2d(Inches.of(43.502), Inches.of(39.553)), new Rotation2d(Degrees.of(234)));
    public static final Pose2d B_HP_LEFT_CENTER = new Pose2d (new Translation2d(Inches.of(43.502), Inches.of(277.447)), new Rotation2d(Degrees.of(486)));
    
    public static final Pose2d R_HP_LEFT_OUT = new Pose2d (new Translation2d(Inches.of(629.58), Inches.of(26.622)), new Rotation2d(Degrees.of(306)));
    public static final Pose2d R_HP_RIGHT_OUT = new Pose2d (new Translation2d(Inches.of(629.58), Inches.of(290.378)), new Rotation2d(Degrees.of(414)));
    public static final Pose2d B_HP_RIGHT_OUT = new Pose2d (new Translation2d(Inches.of(61.3), Inches.of(26.622)), new Rotation2d(Degrees.of(234)));
    public static final Pose2d B_HP_LEFT_OUT = new Pose2d (new Translation2d(Inches.of(61.3), Inches.of(290.378)), new Rotation2d(Degrees.of(486)));
    
    public static final Pose2d R_HP_LEFT_IN = new Pose2d (new Translation2d(Inches.of(665.176), Inches.of(52.484)), new Rotation2d(Degrees.of(306)));
    public static final Pose2d R_HP_RIGHT_IN = new Pose2d (new Translation2d(Inches.of(665.176), Inches.of(264.516)), new Rotation2d(Degrees.of(414)));
    public static final Pose2d B_HP_RIGHT_IN = new Pose2d (new Translation2d(Inches.of(25.704), Inches.of(52.484)), new Rotation2d(Degrees.of(234)));
    public static final Pose2d B_HP_LEFT_IN = new Pose2d (new Translation2d(Inches.of(25.704), Inches.of(264.516)), new Rotation2d(Degrees.of(486)));
    
    public static final Pose2d RED_A = new Pose2d (new Translation2d(Inches.of(563.87), Inches.of(152)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d RED_B = new Pose2d (new Translation2d(Inches.of(563.87), Inches.of(165)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d RED_C = new Pose2d (new Translation2d(Inches.of(544.619), Inches.of(198.302)), new Rotation2d(Degrees.of(60)));
    public static final Pose2d RED_D = new Pose2d (new Translation2d(Inches.of(533.361), Inches.of(204.802)), new Rotation2d(Degrees.of(60)));
    public static final Pose2d RED_E = new Pose2d (new Translation2d(Inches.of(494.899), Inches.of(204.802)), new Rotation2d(Degrees.of(120)));
    public static final Pose2d RED_F = new Pose2d (new Translation2d(Inches.of(483.641), Inches.of(198.302)), new Rotation2d(Degrees.of(120)));
    public static final Pose2d RED_G = new Pose2d (new Translation2d(Inches.of(464.39), Inches.of(165)), new Rotation2d(Degrees.of(180)));
    public static final Pose2d RED_H = new Pose2d (new Translation2d(Inches.of(464.39), Inches.of(152)), new Rotation2d(Degrees.of(180)));
    public static final Pose2d RED_I = new Pose2d (new Translation2d(Inches.of(483.641), Inches.of(118.698)), new Rotation2d(Degrees.of(240)));
    public static final Pose2d RED_J = new Pose2d (new Translation2d(Inches.of(494.899), Inches.of(112.198)), new Rotation2d(Degrees.of(240)));
    public static final Pose2d RED_K = new Pose2d (new Translation2d(Inches.of(533.361), Inches.of(112.198)), new Rotation2d(Degrees.of(300)));
    public static final Pose2d RED_L = new Pose2d (new Translation2d(Inches.of(544.619), Inches.of(118.698)), new Rotation2d(Degrees.of(300)));
    
    public static final Pose2d BLUE_A = new Pose2d (new Translation2d(Inches.of(127), Inches.of(165)), new Rotation2d(Degrees.of(180)));
    public static final Pose2d BLUE_B = new Pose2d (new Translation2d(Inches.of(127), Inches.of(152)), new Rotation2d(Degrees.of(180)));
    public static final Pose2d BLUE_C = new Pose2d (new Translation2d(Inches.of(146.261), Inches.of(118.698)), new Rotation2d(Degrees.of(240)));
    public static final Pose2d BLUE_D = new Pose2d (new Translation2d(Inches.of(157.519), Inches.of(112.198)), new Rotation2d(Degrees.of(240)));
    public static final Pose2d BLUE_E = new Pose2d (new Translation2d(Inches.of(195.971), Inches.of(112.198)), new Rotation2d(Degrees.of(300)));
    public static final Pose2d BLUE_F = new Pose2d (new Translation2d(Inches.of(207.229), Inches.of(118.698)), new Rotation2d(Degrees.of(300)));
    public static final Pose2d BLUE_G = new Pose2d (new Translation2d(Inches.of(226.49), Inches.of(152)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d BLUE_H = new Pose2d (new Translation2d(Inches.of(226.49), Inches.of(165)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d BLUE_I = new Pose2d (new Translation2d(Inches.of(207.229), Inches.of(198.302)), new Rotation2d(Degrees.of(60)));
    public static final Pose2d BLUE_J = new Pose2d (new Translation2d(Inches.of(195.971), Inches.of(204.802)), new Rotation2d(Degrees.of(60)));
    public static final Pose2d BLUE_K = new Pose2d (new Translation2d(Inches.of(157.519), Inches.of(204.802)), new Rotation2d(Degrees.of(120)));
    public static final Pose2d BLUE_L = new Pose2d (new Translation2d(Inches.of(146.261), Inches.of(198.302)), new Rotation2d(Degrees.of(120)));
        
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
}
