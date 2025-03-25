// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbPivotSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbOutCommand extends Command {
  private final ClimbPivotSubsystem climbPivotSubsystem;
  /** Creates a new ClimbCommand. */
  public ClimbOutCommand(ClimbPivotSubsystem climbPivotSubsystem) {
    addRequirements(climbPivotSubsystem);
    this.climbPivotSubsystem = climbPivotSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbPivotSubsystem.setVoltage(7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbPivotSubsystem.stopPivotMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climbPivotSubsystem.isOutPosition();
  }
}
