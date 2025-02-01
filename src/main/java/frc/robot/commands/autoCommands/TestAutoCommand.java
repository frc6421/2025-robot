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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAutoCommand extends SequentialCommandGroup {
  /** Creates a new RedHRB. */
  // subsystems
  private CommandSwerveDrivetrain driveSubsystem;

  private Field2d field;

  public SwerveDriveKinematics kinematics;

  public TestAutoCommand(CommandSwerveDrivetrain drive) {
    
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

    Trajectory toReefTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        TrajectoryConstants.B_HP_LEFT_CENTER, 
        TrajectoryConstants.BLUE_A), 
        reverseConfig);


  // Simulation
    //  field = new Field2d();

    //  if (RobotBase.isSimulation()) {
    //     SmartDashboard.putData(field);

    //     field.setRobotPose(toReefTrajectory.getInitialPose());
      
    //     field.getObject("1b Trajectory").setTrajectory(toReefTrajectory);
    //   }

   
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(() -> driveSubsystem.getPigeon2().setYaw(toReefTrajectory.getInitialPose().getRotation().getDegrees()), driveSubsystem),
        Commands.runOnce(() -> driveSubsystem.resetPose(toReefTrajectory.getInitialPose()), driveSubsystem),
        driveSubsystem.runTrajectoryCommand(toReefTrajectory),
        Commands.runOnce(() -> driveSubsystem.applyRequest(() -> new ApplyRobotSpeeds()), driveSubsystem)
    );
  }
}
