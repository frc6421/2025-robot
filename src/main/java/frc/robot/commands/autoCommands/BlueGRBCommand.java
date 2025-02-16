// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueGRBCommand extends SequentialCommandGroup {
  /** Creates a new RedHRB. */
  // subsystems
  private CommandSwerveDrivetrain driveSubsystem;

  public BlueGRBCommand(CommandSwerveDrivetrain drive) {

    driveSubsystem = drive;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      driveSubsystem.reefAlignCommand(() -> TrajectoryConstants.BLUE_G)
    );
  }
}
