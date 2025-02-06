// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CageIntakeSubsystem;
import frc.robot.subsystems.ClimbPivotSubsystem;
import frc.robot.subsystems.CageIntakeSubsystem.CageIntakeConstants;
import frc.robot.subsystems.ClimbPivotSubsystem.ClimbPivotConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbCommand extends Command {
  private final ClimbPivotSubsystem climbPivotSubsystem;
  private final CageIntakeSubsystem cageIntakeSubsystem;
  /** Creates a new climbCommand. */
  public ClimbCommand(ClimbPivotSubsystem climbPivotSubsystem, CageIntakeSubsystem cageIntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climbPivotSubsystem = climbPivotSubsystem;
    this.cageIntakeSubsystem = cageIntakeSubsystem;
    addRequirements(cageIntakeSubsystem, climbPivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbPivotSubsystem.setVoltageCommand(4).until(() -> climbPivotSubsystem.isOutPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cageIntakeSubsystem.cageIntakeInSpeedCommand();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbPivotSubsystem.setVoltageCommand(-4).until(() -> climbPivotSubsystem.isInPosition());

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cageIntakeSubsystem.haveCage();
  }
}
