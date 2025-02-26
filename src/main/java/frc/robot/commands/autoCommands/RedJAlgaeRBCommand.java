// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
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
        //scoreSequence
        //algae
        new ParallelCommandGroup(
          driveSubsystem.reefAlignCommand(() -> TrajectoryConstants.RED_J),
          intakeSubsystem.setIntakeSpeed(0.1),
          elevatorSubsystem.setElevatorPositionCommand(() -> ElevatorConstants.L2_POSITION.magnitude()),
          new SequentialCommandGroup(new WaitCommand(0.1), wristSubsystem.setAngle(WristConstants.WRIST_SCORE_POSITION.magnitude()))),
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
        driveSubsystem.sourceAlignCommand(() -> TrajectoryConstants.R_HP_LEFT_OUT), 
        wristSubsystem.setAngle(WristConstants.WRIST_INTAKE_POSITION.magnitude()),
        elevatorSubsystem.setElevatorPositionCommand(() -> ElevatorConstants.MIN_HEIGHT_MATCH),
        intakeSubsystem.intakeCoral()),

        //intakeSequence
			intakeSubsystem.stopIntake(), 
     
      new ParallelCommandGroup(
          new SequentialCommandGroup(driveSubsystem.offsetAlignCommand(() -> TrajectoryConstants.RED_J_OFFSET), driveSubsystem.sourceAlignCommand(() -> TrajectoryConstants.RED_J)),
          intakeSubsystem.setIntakeSpeed(0.1),
          elevatorSubsystem.setElevatorPositionCommand(() -> ElevatorConstants.L3_POSITION.magnitude()),
          new SequentialCommandGroup(new WaitCommand(0.1), wristSubsystem.setAngle(WristConstants.WRIST_SCORE_POSITION.magnitude()))),
      intakeSubsystem.setIntakeSpeed(0.3),
      new WaitCommand(0.1),
      intakeSubsystem.stopIntake(),
      intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_OUT_SPEED),
      new WaitCommand(0.1), 
      intakeSubsystem.stopIntake(),
		
      new ParallelCommandGroup(
        driveSubsystem.sourceAlignCommand(() -> TrajectoryConstants.R_HP_LEFT_OUT), 
        wristSubsystem.setAngle(WristConstants.WRIST_INTAKE_POSITION.magnitude()),
        elevatorSubsystem.setElevatorPositionCommand(() -> ElevatorConstants.MIN_HEIGHT_MATCH),
        intakeSubsystem.intakeCoral()),

        new ParallelCommandGroup(
          new SequentialCommandGroup(driveSubsystem.offsetAlignCommand(() -> TrajectoryConstants.RED_J_OFFSET), driveSubsystem.sourceAlignCommand(() -> TrajectoryConstants.RED_I)),
          intakeSubsystem.setIntakeSpeed(0.1),
          elevatorSubsystem.setElevatorPositionCommand(() -> ElevatorConstants.L3_POSITION.magnitude()),
          new SequentialCommandGroup(new WaitCommand(0.1), wristSubsystem.setAngle(WristConstants.WRIST_SCORE_POSITION.magnitude()))),
      
          intakeSubsystem.setIntakeSpeed(0.3),
          new WaitCommand(0.1),
          intakeSubsystem.stopIntake(),
          intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_OUT_SPEED),
          new WaitCommand(0.2),
          intakeSubsystem.stopIntake(), 

          new ParallelCommandGroup(
        wristSubsystem.setAngle(WristConstants.WRIST_INTAKE_POSITION.magnitude()),
        elevatorSubsystem.setElevatorPositionCommand(() -> ElevatorConstants.MIN_HEIGHT_MATCH))
          );
  }
}
