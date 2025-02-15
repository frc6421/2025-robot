// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueJKLBBCommand extends SequentialCommandGroup {
  /** Creates a new RedHRB. */
  // subsystems
  private CommandSwerveDrivetrain driveSubsystem;

  private Field2d field;

  public SwerveDriveKinematics kinematics;

  public BlueJKLBBCommand(CommandSwerveDrivetrain drive) {
    
    driveSubsystem = drive;

    kinematics = driveSubsystem.getKinematics();

    TrajectoryConfig forwardConfig = new TrajectoryConfig(
        (AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND) - 2,
        (AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED) - 2)
        .setKinematics(kinematics);
    
    TrajectoryConfig reverseConfig = new TrajectoryConfig(
        (AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND) - 2,
        (AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED) - 2)
        .setKinematics(kinematics)
        .setReversed(true);

    Trajectory toJTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        TrajectoryConstants.BLUE_BB_START, 
        TrajectoryConstants.BLUE_J), 
        reverseConfig);

    Trajectory toCoral1Trajectory = TrajectoryGenerator.generateTrajectory(List.of(
        TrajectoryConstants.BLUE_J, 
        TrajectoryConstants.B_HP_LEFT_OUT), 
        forwardConfig);

    Trajectory toKTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        TrajectoryConstants.B_HP_LEFT_OUT, 
        TrajectoryConstants.BLUE_K), 
        reverseConfig);

    Trajectory toCoral2Trajectory = TrajectoryGenerator.generateTrajectory(List.of(
        TrajectoryConstants.BLUE_K, 
        TrajectoryConstants.B_HP_LEFT_OUT), 
        forwardConfig);

    Trajectory toLTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        TrajectoryConstants.B_HP_LEFT_OUT, 
        TrajectoryConstants.BLUE_L), 
        reverseConfig);

    Trajectory toCoral3Trajectory = TrajectoryGenerator.generateTrajectory(List.of(
        TrajectoryConstants.BLUE_L, 
        TrajectoryConstants.B_HP_LEFT_OUT), 
        forwardConfig);

    
  // Simulation
    //  field = new Field2d();

    //  if (RobotBase.isSimulation()) {
    //     SmartDashboard.putData(field);

    //     field.setRobotPose(toJTrajectory.getInitialPose());
      
    //     field.getObject("1 Trajectory").setTrajectory(toJTrajectory);
    //     field.getObject("2 Trajectory").setTrajectory(toCoral1Trajectory);
    //     field.getObject("3 Trajectory").setTrajectory(toKTrajectory);
    //     field.getObject("4 Trajectory").setTrajectory(toCoral2Trajectory);
    //     field.getObject("5 Trajectory").setTrajectory(toLTrajectory);
    //     field.getObject("6 Trajectory").setTrajectory(toCoral3Trajectory);
    //   }


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        driveSubsystem.reefAlignCommand(() -> TrajectoryConstants.BLUE_J),
        driveSubsystem.sourceAlignCommand(() -> TrajectoryConstants.B_HP_LEFT_OUT), 
        driveSubsystem.reefAlignCommand(() -> TrajectoryConstants.BLUE_K), 
        driveSubsystem.sourceAlignCommand(() -> TrajectoryConstants.B_HP_LEFT_OUT), 
        driveSubsystem.reefAlignCommand(() -> TrajectoryConstants.BLUE_L)
    );
  }
}