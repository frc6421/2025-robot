// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorConstants;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.WristSubsystem.WristConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedJAlgaeRBCommand extends SequentialCommandGroup {
  /** Creates a new BLUEHRB. */
  // subsystems
  private CommandSwerveDrivetrain driveSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private WristSubsystem wristSubsystem;
  private IntakeSubsystem intakeSubsystem;


  public RedJAlgaeRBCommand(CommandSwerveDrivetrain drive, ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake) {

    driveSubsystem = drive;
    elevatorSubsystem = elevator;
    wristSubsystem = wrist;
    intakeSubsystem = intake;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        driveSubsystem.reefAlignCommand(() -> TrajectoryConstants.RED_J),

        //scoreSequence
      new ParallelCommandGroup(
          elevatorSubsystem.setElevatorPositionCommand(() -> ElevatorConstants.L2_POSITION.magnitude()),
          new SequentialCommandGroup(new WaitCommand(0.3), wristSubsystem.setAngle(WristConstants.WRIST_SCORE_POSITION.magnitude()))),
      intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_OUT_SPEED),
      new WaitCommand(0.2),
      intakeSubsystem.stopIntake(),
      new ParallelCommandGroup(
        wristSubsystem.setAngle(WristConstants.WRIST_INTAKE_POSITION.magnitude()),
        new SequentialCommandGroup(new WaitCommand(0.4), elevatorSubsystem.setElevatorPositionCommand(() -> ElevatorConstants.MIN_HEIGHT_MATCH))),

        //algae
        new ParallelCommandGroup(
				elevatorSubsystem.setElevatorPositionCommand(() -> ElevatorConstants.L3_POSITION.magnitude()),
				new SequentialCommandGroup(new WaitCommand(0.2), intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_OUT_SPEED), wristSubsystem.setAngle(WristConstants.WRIST_ALGAE_POSITION.magnitude()))),
        intakeSubsystem.stopIntake(),
        new ParallelCommandGroup(
          wristSubsystem.setAngle(WristConstants.WRIST_INTAKE_POSITION.magnitude()),
          new SequentialCommandGroup(new WaitCommand(0.2), elevatorSubsystem.setElevatorPositionCommand(() -> ElevatorConstants.MIN_HEIGHT_MATCH))),
      

      driveSubsystem.sourceAlignCommand(() -> TrajectoryConstants.R_HP_LEFT_CENTER), 

        //intakeSequence
			intakeSubsystem.intakeCoral(),
			intakeSubsystem.stopIntake(),

      driveSubsystem.reefAlignCommand(() -> TrajectoryConstants.RED_A)
      );
  }
}
