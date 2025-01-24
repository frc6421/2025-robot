// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedJKLRB extends SequentialCommandGroup {
  /** Creates a new RedHRB. */
  // subsystems
  private CommandSwerveDrivetrain driveSubsystem;

  private Field2d field;

  public SwerveDriveKinematics kinematics;

  public RedJKLRB(CommandSwerveDrivetrain drive) {
    
    driveSubsystem = drive;

    kinematics = driveSubsystem.getKinematics();



    TrajectoryConfig forwardConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(kinematics);
    
    TrajectoryConfig reverseConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(kinematics)
        .setReversed(true);

    Trajectory toJTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        TrajectoryConstants.RED_RB_START, 
        TrajectoryConstants.RED_J), 
        reverseConfig);

    Trajectory toCoral1Trajectory = TrajectoryGenerator.generateTrajectory(List.of(
        TrajectoryConstants.RED_J, 
        TrajectoryConstants.R_HP_LEFT_OUT), 
        forwardConfig);

    Trajectory toKTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        TrajectoryConstants.R_HP_LEFT_OUT, 
        TrajectoryConstants.RED_K), 
        reverseConfig);

    Trajectory toCoral2Trajectory = TrajectoryGenerator.generateTrajectory(List.of(
        TrajectoryConstants.RED_K, 
        TrajectoryConstants.R_HP_LEFT_OUT), 
        forwardConfig);

    Trajectory toLTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        TrajectoryConstants.R_HP_LEFT_OUT, 
        TrajectoryConstants.RED_L), 
        reverseConfig);

    Trajectory toCoral3Trajectory = TrajectoryGenerator.generateTrajectory(List.of(
        TrajectoryConstants.RED_L, 
        TrajectoryConstants.R_HP_LEFT_OUT), 
        forwardConfig);

    

  // Simulation
     field = new Field2d();

     if (RobotBase.isSimulation()) {
        SmartDashboard.putData(field);

        field.setRobotPose(toJTrajectory.getInitialPose());
      
        field.getObject("first Trajectory").setTrajectory(toJTrajectory);
        field.getObject("second Trajectory").setTrajectory(toCoral1Trajectory);
        field.getObject("third Trajectory").setTrajectory(toKTrajectory);
        field.getObject("fourth Trajectory").setTrajectory(toCoral2Trajectory);
        field.getObject("fifth Trajectory").setTrajectory(toLTrajectory);
        field.getObject("sixth Trajectory").setTrajectory(toCoral3Trajectory);
      }


    var thetaController = new ProfiledPIDController(
        AutoConstants.THETA_P, AutoConstants.THETA_I, AutoConstants.THETA_D,
        new TrapezoidProfile.Constraints(AutoConstants.AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
            AutoConstants.AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
        // Position controllers
        new PIDController(AutoConstants.X_DRIVE_P, AutoConstants.X_DRIVE_I, AutoConstants.X_DRIVE_D),
        new PIDController(AutoConstants.Y_DRIVE_P, AutoConstants.Y_DRIVE_I, AutoConstants.Y_DRIVE_D),
        thetaController);


   
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
