// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LED_NOT_a_Subsystem;
import frc.robot.LED_NOT_a_Subsystem.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.WristSubsystem.WristConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreFinishCommand extends SequentialCommandGroup {
  /** Creates a new ScorePrepCommand. */

  private ElevatorSubsystem elevatorSubsystem;
  private WristSubsystem wristSubsystem;
  private IntakeSubsystem intakeSubsystem;

  private final WristCommand wristIntakeCommand;

  public ScoreFinishCommand(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake, LED_NOT_a_Subsystem led) {

    elevatorSubsystem = elevator;
    wristSubsystem = wrist;
    intakeSubsystem = intake;

    wristIntakeCommand = new WristCommand(wristSubsystem, WristConstants.WRIST_INTAKE_POSITION.magnitude());	

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    intakeSubsystem.setIntakeSpeed(0.3),
    new WaitCommand(0.1),
    intakeSubsystem.stopIntake(),
    intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_OUT_SPEED),
    led.setLED(LEDConstants.BLUE, true),
		new WaitCommand(0.1),
    intakeSubsystem.stopIntake(),
		new ParallelCommandGroup(
			wristIntakeCommand,
			elevatorSubsystem.setElevatorPositionCommand(() -> (ElevatorConstants.MIN_HEIGHT_MATCH))),
    led.off());
  }
}
