// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;
import frc.robot.subsystems.WristSubsystem.WristConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeRemovalCommand extends SequentialCommandGroup {
  /** Creates a new ScoreSequenceCommand. */
  private ElevatorSubsystem elevatorSubsystem;
  private WristSubsystem wristSubsystem;
  private IntakeSubsystem intakeSubsystem;

  private final WristCommand wristAlgae2Command;
  private final WristCommand wristAlgae3Command;
  
  public AlgaeRemovalCommand(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake, DoubleSupplier position) {

    elevatorSubsystem = elevator;
    wristSubsystem = wrist;
    intakeSubsystem = intake;

    wristAlgae2Command = new WristCommand(wristSubsystem, WristConstants.WRIST_ALGAE_POSITION.magnitude());	
    wristAlgae3Command = new WristCommand(wristSubsystem, 16);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ParallelCommandGroup(
        new ConditionalCommand(elevatorSubsystem.setElevatorPositionCommand(() -> 39), elevatorSubsystem.setElevatorPositionCommand(() -> 58), () -> (position.getAsDouble() == ElevatorConstants.L2_POSITION.magnitude())),
				intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_OUT_SPEED_L4), 
        new ConditionalCommand(wristAlgae3Command, wristAlgae2Command, () -> (position.getAsDouble() == ElevatorConstants.L3_POSITION.magnitude())))
    );
  }
}