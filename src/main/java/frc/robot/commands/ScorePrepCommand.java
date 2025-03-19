// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.WristSubsystem.WristConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScorePrepCommand extends SequentialCommandGroup {
  /** Creates a new ScorePrepCommand. */

  private ElevatorSubsystem elevatorSubsystem;
  private WristSubsystem wristSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private CommandSwerveDrivetrain drivetrain;

  private final WristCommand wristScoreCommand;
	private final WristCommand wristScore4Command;

  public ScorePrepCommand(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake,
      CommandSwerveDrivetrain drive, DoubleSupplier position, Supplier<Pose2d> location) {

    elevatorSubsystem = elevator;
    wristSubsystem = wrist;
    intakeSubsystem = intake;
    drivetrain = drive;

    wristScoreCommand = new WristCommand(wristSubsystem, WristConstants.WRIST_SCORE_POSITION.magnitude());
		wristScore4Command = new WristCommand(wristSubsystem, WristConstants.WRIST_SCORE_POSITION_4.magnitude());	

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            drivetrain.reefAlignCommand(location),
            intakeSubsystem.setIntakeSpeed(0.1),
            elevatorSubsystem.setElevatorPositionCommand(position),
            new ConditionalCommand(wristScore4Command, wristScoreCommand, () -> (position.getAsDouble() == ElevatorConstants.L4_POSITION.magnitude()))),
        intakeSubsystem.setIntakeSpeed(0.3),
        new WaitCommand(0.1),
        intakeSubsystem.stopIntake());
  }
}
