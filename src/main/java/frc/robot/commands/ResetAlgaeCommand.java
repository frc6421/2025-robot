// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorConstants;
import frc.robot.subsystems.WristSubsystem.WristConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetAlgaeCommand extends SequentialCommandGroup {
  /** Creates a new ScoreSequenceCommand. */
  private ElevatorSubsystem elevatorSubsystem;
  private WristSubsystem wristSubsystem;
  private IntakeSubsystem intakeSubsystem;
  
  public ResetAlgaeCommand(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake) {

    elevatorSubsystem = elevator;
    wristSubsystem = wrist;
    intakeSubsystem = intake;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      intakeSubsystem.stopIntake(),
		  new ParallelCommandGroup(
			  wristSubsystem.setAngle(WristConstants.WRIST_INTAKE_POSITION.magnitude()),
			  elevatorSubsystem.setElevatorPositionCommand(() -> ElevatorConstants.MIN_HEIGHT_MATCH)));
  }
}