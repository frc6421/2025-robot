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
public class BlueEAlgaeRBCommand extends SequentialCommandGroup {
  /** Creates a new BLUEHRB. */
  // subsystems
  private CommandSwerveDrivetrain driveSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private WristSubsystem wristSubsystem;
  private IntakeSubsystem intakeSubsystem;


  public BlueEAlgaeRBCommand(CommandSwerveDrivetrain drive, ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake) {

    driveSubsystem = drive;
    elevatorSubsystem = elevator;
    wristSubsystem = wrist;
    intakeSubsystem = intake;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        driveSubsystem.reefAlignCommand(() -> TrajectoryConstants.BLUE_E),
        intakeSubsystem.setIntakeSpeed(0.1),
        elevatorSubsystem.setElevatorPositionCommand(() -> ElevatorConstants.L2_POSITION.magnitude()),
        wristSubsystem.setAngle(WristConstants.WRIST_SCORE_POSITION.magnitude())),
    intakeSubsystem.setIntakeSpeed(0.3),
    new WaitCommand(0.1),
    intakeSubsystem.stopIntake(),
    intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_OUT_SPEED),
    new WaitCommand(0.1),

    new ParallelCommandGroup(
        wristSubsystem.setAngle(WristConstants.WRIST_ALGAE_POSITION.magnitude()),
        elevatorSubsystem.setElevatorPositionCommand(() -> ElevatorConstants.L3_POSITION.magnitude())),
    intakeSubsystem.stopIntake(),

    new ParallelCommandGroup(
        driveSubsystem.sourceAlignCommand(() -> TrajectoryConstants.B_HP_RIGHT_CENTER),
        wristSubsystem.setAngle(WristConstants.WRIST_INTAKE_POSITION.magnitude()),
        elevatorSubsystem.setElevatorPositionCommand(() -> ElevatorConstants.MIN_HEIGHT_MATCH),
        intakeSubsystem.intakeCoral()),

    intakeSubsystem.stopIntake(),

    new ParallelCommandGroup(
        new SequentialCommandGroup(driveSubsystem.offsetAlignCommand(() -> TrajectoryConstants.BLUE_E_OFFSET),
            driveSubsystem.reefAlignCommand(() -> TrajectoryConstants.BLUE_E)),
        intakeSubsystem.setIntakeSpeed(0.1),
        elevatorSubsystem.setElevatorPositionCommand(() -> ElevatorConstants.L3_POSITION.magnitude()),
        wristSubsystem.setAngle(WristConstants.WRIST_SCORE_POSITION.magnitude())),
    intakeSubsystem.setIntakeSpeed(0.3),
    new WaitCommand(0.1),
    intakeSubsystem.stopIntake(),
    intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_OUT_SPEED),
    new WaitCommand(0.1),
    intakeSubsystem.stopIntake(),

    new ParallelCommandGroup(
        driveSubsystem.sourceAlignCommand(() -> TrajectoryConstants.B_HP_RIGHT_CENTER),
        wristSubsystem.setAngle(WristConstants.WRIST_INTAKE_POSITION.magnitude()),
        elevatorSubsystem.setElevatorPositionCommand(() -> ElevatorConstants.MIN_HEIGHT_MATCH),
        intakeSubsystem.intakeCoral()),

    intakeSubsystem.stopIntake(),

    new ParallelCommandGroup(
        new SequentialCommandGroup(driveSubsystem.offsetAlignCommand(() -> TrajectoryConstants.BLUE_E_OFFSET),
            driveSubsystem.reefAlignCommand(() -> TrajectoryConstants.BLUE_F)),
        intakeSubsystem.setIntakeSpeed(0.1),
        elevatorSubsystem.setElevatorPositionCommand(() -> ElevatorConstants.L3_POSITION.magnitude()),
        wristSubsystem.setAngle(WristConstants.WRIST_SCORE_POSITION.magnitude())),

    intakeSubsystem.setIntakeSpeed(0.3),
    new WaitCommand(0.1),
    intakeSubsystem.stopIntake(),
    intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_OUT_SPEED),
    new WaitCommand(0.1),
    intakeSubsystem.stopIntake(),

    new ParallelCommandGroup(
        wristSubsystem.setAngle(WristConstants.WRIST_INTAKE_POSITION.magnitude()),
        elevatorSubsystem.setElevatorPositionCommand(() -> ElevatorConstants.MIN_HEIGHT_MATCH)));
  }
}
