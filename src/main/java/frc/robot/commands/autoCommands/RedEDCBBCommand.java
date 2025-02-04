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
public class RedEDCBBCommand extends SequentialCommandGroup {
  /** Creates a new RedHRB. */
  // subsystems
  private CommandSwerveDrivetrain driveSubsystem;

  private Field2d field;

  public SwerveDriveKinematics kinematics;

  public RedEDCBBCommand(CommandSwerveDrivetrain drive) {
    
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

    Trajectory toETrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        TrajectoryConstants.RED_BB_START, 
        TrajectoryConstants.RED_E), 
        reverseConfig);

    Trajectory toCoral1Trajectory = TrajectoryGenerator.generateTrajectory(List.of(
        TrajectoryConstants.RED_E, 
        TrajectoryConstants.R_HP_RIGHT_OUT), 
        forwardConfig);

    Trajectory toDTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        TrajectoryConstants.R_HP_RIGHT_OUT, 
        TrajectoryConstants.RED_D), 
        reverseConfig);

    Trajectory toCoral2Trajectory = TrajectoryGenerator.generateTrajectory(List.of(
        TrajectoryConstants.RED_D, 
        TrajectoryConstants.R_HP_RIGHT_OUT), 
        forwardConfig);

    Trajectory toCTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        TrajectoryConstants.R_HP_RIGHT_OUT, 
        TrajectoryConstants.RED_C), 
        reverseConfig);

    Trajectory toCoral3Trajectory = TrajectoryGenerator.generateTrajectory(List.of(
        TrajectoryConstants.RED_C, 
        TrajectoryConstants.R_HP_RIGHT_OUT), 
        forwardConfig);

    

  // Simulation
    //  field = new Field2d();

    //  if (RobotBase.isSimulation()) {
    //     SmartDashboard.putData(field);

    //     field.setRobotPose(toETrajectory.getInitialPose());
      
    //     field.getObject("1 Trajectory").setTrajectory(toETrajectory);
    //     field.getObject("2 Trajectory").setTrajectory(toCoral1Trajectory);
    //     field.getObject("3 Trajectory").setTrajectory(toDTrajectory);
    //     field.getObject("4 Trajectory").setTrajectory(toCoral2Trajectory);
    //     field.getObject("5 Trajectory").setTrajectory(toCTrajectory);
    //     field.getObject("6 Trajectory").setTrajectory(toCoral3Trajectory);
    //   }
    // System.out.println("TIME: " + (toETrajectory.getTotalTimeSeconds() + toCoral1Trajectory.getTotalTimeSeconds() + toDTrajectory.getTotalTimeSeconds() + toCoral2Trajectory.getTotalTimeSeconds() + toCTrajectory.getTotalTimeSeconds() + toCoral3Trajectory.getTotalTimeSeconds()));
   
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
