// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorConstants;
import frc.robot.subsystems.WristSubsystem.WristConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreSequenceCommand extends SequentialCommandGroup {
  /** Creates a new ScoreSequenceCommand. */
  private ElevatorSubsystem elevatorSubsystem;
  private WristSubsystem wristSubsystem;
  private IntakeSubsystem intakeSubsystem;
  
  public ScoreSequenceCommand(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake, DoubleSupplier position) {

    elevatorSubsystem = elevator;
    wristSubsystem = wrist;
    intakeSubsystem = intake;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ParallelCommandGroup(
				elevatorSubsystem.setElevatorPositionCommand(position),
				new SequentialCommandGroup(new WaitCommand(0.5), wristSubsystem.setAngle(WristConstants.WRIST_SCORE_POSITION.magnitude()))),
		intakeSubsystem.runOnce(() -> intakeSubsystem.setIntakeOutSpeed()),
		new WaitCommand(0.3),
		intakeSubsystem.runOnce(() -> intakeSubsystem.stopIntake()),
		new ParallelCommandGroup(
			wristSubsystem.setAngle(WristConstants.WRIST_INTAKE_POSITION.magnitude()),
			new SequentialCommandGroup(new WaitCommand(0.8), elevatorSubsystem.setElevatorPositionCommand(() -> Units.metersToInches(ElevatorConstants.MIN_HEIGHT))))
    );
  }
}
