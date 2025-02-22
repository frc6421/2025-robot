// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IntakeSequenceCommand;
import frc.robot.commands.ScoreSequenceCommand;
import frc.robot.commands.autoCommands.BlueEDCRBCommand;
import frc.robot.commands.autoCommands.BlueGRBCommand;
import frc.robot.commands.autoCommands.BlueJKLBBCommand;
import frc.robot.commands.autoCommands.RedEDCBBCommand;
import frc.robot.commands.autoCommands.RedHRBCommand;
import frc.robot.commands.autoCommands.RedJKLRBCommand;
import frc.robot.commands.autoCommands.TestAutoCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CageIntakeSubsystem;
import frc.robot.subsystems.ClimbPivotSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorConstants;
import frc.robot.subsystems.WristSubsystem.WristConstants;

public class RobotContainer {
	private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
	private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
																						// max angular velocity
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

	/* Setting up bindings for necessary control of the swerve drive platform */
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.03) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	private final Telemetry logger = new Telemetry(MaxSpeed);

	private final CommandXboxController joystick = new CommandXboxController(0);

	private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	private final WristSubsystem wristSubsystem = new WristSubsystem();
	private final ClimbPivotSubsystem climbSubsystem = new ClimbPivotSubsystem();
	//private final CageIntakeSubsystem cageIntakeSubsystem = new CageIntakeSubsystem();

	//private final ClimbCommand climbCommand = new ClimbCommand(climbSubsystem, cageIntakeSubsystem);

	private final SlewRateLimiter xDriveSlew = new SlewRateLimiter(Constants.DriveConstants.DRIVE_SLEW_RATE);
	private final SlewRateLimiter yDriveSlew = new SlewRateLimiter(Constants.DriveConstants.DRIVE_SLEW_RATE);

	private final TestAutoCommand testAuto;
	private final RedJKLRBCommand redJKLRB;
	private final RedEDCBBCommand redEDCBB;
	private final RedHRBCommand redHRB;
	private final BlueJKLBBCommand blueJKLBB;
	private final BlueEDCRBCommand blueEDCRB;
	private final BlueGRBCommand blueGRB;

	private SendableChooser<Command> autoChooser;
    private SendableChooser<Pose2d> redPositionChooser;
    private SendableChooser<Pose2d> bluePositionChooser;
    private SendableChooser<Pose2d> redSourceChooser;
    private SendableChooser<Pose2d> blueSourceChooser;
	private SendableChooser<Double> elevatorPositionChooser;

	private final ScoreSequenceCommand scoreSequenceCommand;
	private final IntakeSequenceCommand intakeSequenceCommand;

	public RobotContainer() {

		elevatorPositionChooser = new SendableChooser<>();
		elevatorPositionChooser.setDefaultOption("L1", ElevatorConstants.L1_POSITION.magnitude());
		elevatorPositionChooser.addOption("L2", ElevatorConstants.L2_POSITION.magnitude());
		elevatorPositionChooser.addOption("L3", ElevatorConstants.L3_POSITION.magnitude());
		elevatorPositionChooser.addOption("L4", ElevatorConstants.L4_POSITION.magnitude());

		scoreSequenceCommand = new ScoreSequenceCommand(elevatorSubsystem, wristSubsystem, intakeSubsystem, () -> getElevatorPosition());
		intakeSequenceCommand = new IntakeSequenceCommand(elevatorSubsystem, wristSubsystem, intakeSubsystem);


		testAuto = new TestAutoCommand(drivetrain);
		redJKLRB = new RedJKLRBCommand(drivetrain, elevatorSubsystem, wristSubsystem, intakeSubsystem);
		redEDCBB = new RedEDCBBCommand(drivetrain);
		redHRB = new RedHRBCommand(drivetrain);
		blueJKLBB = new BlueJKLBBCommand(drivetrain);
		blueEDCRB = new BlueEDCRBCommand(drivetrain, elevatorSubsystem, wristSubsystem, intakeSubsystem);
		blueGRB = new BlueGRBCommand(drivetrain);

		autoChooser = new SendableChooser<>();
		autoChooser.addOption("Auto Test", testAuto);
		autoChooser.addOption("Red JKL RB", redJKLRB);
		autoChooser.addOption("Red EDC BB", redEDCBB);
		autoChooser.addOption("Red H RB", redHRB);
		autoChooser.addOption("Blue JKL BB", blueJKLBB);
		autoChooser.addOption("Blue EDC RB", blueEDCRB);
		autoChooser.addOption("Blue G RB", blueGRB);
        autoChooser = new SendableChooser<>();
        autoChooser.addOption("Auto Test", testAuto);
        autoChooser.addOption("Red JKL RB", redJKLRB);
        autoChooser.addOption("Red EDC BB", redEDCBB);
        autoChooser.addOption("Red H RB", redHRB);
        autoChooser.addOption("Blue JKL BB", blueJKLBB);
        autoChooser.addOption("Blue EDC RB", blueEDCRB);
        autoChooser.addOption("Blue G RB", blueGRB);

        redPositionChooser = new SendableChooser<>();
        redPositionChooser.setDefaultOption("A", TrajectoryConstants.RED_A);
        redPositionChooser.addOption("B", TrajectoryConstants.RED_B);
        redPositionChooser.addOption("C", TrajectoryConstants.RED_C);
        redPositionChooser.addOption("D", TrajectoryConstants.RED_D);
        redPositionChooser.addOption("E", TrajectoryConstants.RED_E);
        redPositionChooser.addOption("F", TrajectoryConstants.RED_F);
        redPositionChooser.addOption("G", TrajectoryConstants.RED_G);
        redPositionChooser.addOption("H", TrajectoryConstants.RED_H);
        redPositionChooser.addOption("I", TrajectoryConstants.RED_I);
        redPositionChooser.addOption("J", TrajectoryConstants.RED_J);
        redPositionChooser.addOption("K", TrajectoryConstants.RED_K);
        redPositionChooser.addOption("L", TrajectoryConstants.RED_L);

        bluePositionChooser = new SendableChooser<>();
        bluePositionChooser.setDefaultOption("A", TrajectoryConstants.BLUE_A);
        bluePositionChooser.addOption("B", TrajectoryConstants.BLUE_B);
        bluePositionChooser.addOption("C", TrajectoryConstants.BLUE_C);
        bluePositionChooser.addOption("D", TrajectoryConstants.BLUE_D);
        bluePositionChooser.addOption("E", TrajectoryConstants.BLUE_E);
        bluePositionChooser.addOption("F", TrajectoryConstants.BLUE_F);
        bluePositionChooser.addOption("G", TrajectoryConstants.BLUE_G);
        bluePositionChooser.addOption("H", TrajectoryConstants.BLUE_H);
        bluePositionChooser.addOption("I", TrajectoryConstants.BLUE_I);
        bluePositionChooser.addOption("J", TrajectoryConstants.BLUE_J);
        bluePositionChooser.addOption("K", TrajectoryConstants.BLUE_K);
        bluePositionChooser.addOption("L", TrajectoryConstants.BLUE_L);

        redSourceChooser = new SendableChooser<>();
        redSourceChooser.setDefaultOption("1", TrajectoryConstants.R_HP_LEFT_OUT);
        redSourceChooser.addOption("2", TrajectoryConstants.R_HP_LEFT_CENTER);
        redSourceChooser.addOption("3", TrajectoryConstants.R_HP_LEFT_IN);
        redSourceChooser.addOption("4", TrajectoryConstants.R_HP_RIGHT_IN);
        redSourceChooser.addOption("5", TrajectoryConstants.R_HP_RIGHT_CENTER);
        redSourceChooser.addOption("6", TrajectoryConstants.R_HP_RIGHT_OUT);

        blueSourceChooser = new SendableChooser<>();
        blueSourceChooser.setDefaultOption("1", TrajectoryConstants.B_HP_LEFT_OUT);
        blueSourceChooser.addOption("2", TrajectoryConstants.B_HP_LEFT_CENTER);
        blueSourceChooser.addOption("3", TrajectoryConstants.B_HP_LEFT_IN);
        blueSourceChooser.addOption("4", TrajectoryConstants.B_HP_RIGHT_IN);
        blueSourceChooser.addOption("5", TrajectoryConstants.B_HP_RIGHT_CENTER);
        blueSourceChooser.addOption("6", TrajectoryConstants.B_HP_RIGHT_OUT);

		SmartDashboard.putData("Auto Chooser", autoChooser);
		SmartDashboard.putData("Position Chooser", redPositionChooser);
		SmartDashboard.putData("Source Chooser", redSourceChooser);
		SmartDashboard.putData("Elevator Position Chooser", elevatorPositionChooser);
		SmartDashboard.putData("Gyro", drivetrain.getPigeon2());

    	configureBindings();
	}

	private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

		// Uncomment for running sysid routines

		//  joystick.a().onTrue(new InstantCommand(() -> SignalLogger.start()));
		//  joystick.b().onTrue(new InstantCommand(() -> SignalLogger.stop()));
		//  joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
		//  joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		//  joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		//  joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));

		// Note that X is defined as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.
		drivetrain.setDefaultCommand(
				// Drivetrain will execute this command periodically
				drivetrain.applyRequest(() -> drive
						// Drive forward with negative Y (forward)
						.withVelocityX(xDriveSlew.calculate(-joystick.getLeftY() * MaxSpeed))
						// Drive left with negative X (left)
						.withVelocityY(yDriveSlew.calculate(-joystick.getLeftX() * MaxSpeed))
						// Drive counterclockwise with negative X (left)
						.withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

		// TODO Do we need / want?
		// joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
		// joystick.b().whileTrue(drivetrain.applyRequest(
		// () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
		// -joystick.getLeftX()))));

		//  joystick.y().whileTrue((wristSubsystem.run(() -> wristSubsystem.setAngle(WristConstants.WRIST_FORWARD_SOFT_LIMIT.magnitude()))));

		//  joystick.x().whileTrue((wristSubsystem.run(() -> wristSubsystem.setAngle(WristConstants.WRIST_INTAKE_POSITION.magnitude()))));

		//  joystick.a().whileTrue(intakeSubsystem.run(() -> intakeSubsystem.setIntakeInSpeed()));
		//  joystick.a().onFalse(intakeSubsystem.runOnce(() -> intakeSubsystem.stopIntake()));

		//  joystick.b().whileTrue(intakeSubsystem.run(() -> intakeSubsystem.setIntakeOutSpeed()));
		//  joystick.b().onFalse(intakeSubsystem.runOnce(() -> intakeSubsystem.stopIntake()));

		// Climb buttons

		// joystick.a().whileTrue(climbSubsystem.setVoltageCommand(2)
		// 		.until(climbSubsystem::isOutPosition)
		// 		.andThen(cageIntakeSubsystem.cageIntakeInSpeedCommand())
		// 		.until(() -> cageIntakeSubsystem.haveCage())
		// 		.finallyDo(() -> climbSubsystem.setVoltageCommand(-2).until(climbSubsystem::isInPosition)));

		// joystick.b().onFalse(climbSubsystem.stopPivotMotors()
		// 		.alongWith(cageIntakeSubsystem.cageIntakeInSpeedCommand()));

		// reset the field-centric heading on left bumper press
		//joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //joystick.x().onTrue(elevatorSubsystem.setElevatorPositionCommand(elevatorSubsystem.getSelectedState()));
        // joystick.b().whileTrue(elevatorSubsystem.setElevatorPositionCommand(29.271 + 36));
        // joystick.a().onFalse(new InstantCommand(() -> elevatorSubsystem.stopElevator()));

        // joystick.b().onFalse(elevatorSubsystem.setElevatorPositionCommand(29.271));
        // joystick.b().onFalse(new InstantCommand(() -> elevatorSubsystem.stopElevator()));

        // joystick.x().whileTrue(elevatorSubsystem.setElevatorVoltage(1));
        // joystick.x().onFalse(new InstantCommand(() -> elevatorSubsystem.stopElevator()));

        // joystick.y().whileTrue(elevatorSubsystem.setElevatorVoltage(-1));
        // joystick.y().onFalse(new InstantCommand(() -> elevatorSubsystem.stopElevator()));

        // joystick.a().whileTrue(elevatorSubsystem.setElevatorPositionCommand(ElevatorConstants.L1_GOAL));
        // joystick.b().whileTrue(elevatorSubsystem.setElevatorPositionCommand(ElevatorConstants.BOTTOM_GOAL));

		joystick.rightBumper().whileTrue(drivetrain.reefAlignCommand(() -> getSelectedPoseCommand()));
		joystick.leftBumper().whileTrue(drivetrain.sourceAlignCommand(() -> getSelectedSource()));

		joystick.leftTrigger().onTrue(intakeSequenceCommand);
		joystick.rightTrigger().onTrue(scoreSequenceCommand);

				joystick.x().onTrue(drivetrain.resetGyro());

				// Manual Overrides
				joystick.start().whileTrue(elevatorSubsystem.resetElevator());
				joystick.back().whileTrue(wristSubsystem.resetWrist());

				joystick.povLeft().onTrue(drivetrain.nudgeCommand(-1));
				joystick.povRight().onTrue(drivetrain.nudgeCommand(1));

		drivetrain.registerTelemetry(logger::telemeterize);
	}

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Pose2d getSelectedPoseCommand() {
			if (DriverStation.getAlliance().equals(Alliance.Red)) {
			return redPositionChooser.getSelected();
			} else {
				return bluePositionChooser.getSelected();
			}
    }

    public Pose2d getSelectedSource() {
			if (DriverStation.getAlliance().equals(Alliance.Red)) {
        return redSourceChooser.getSelected();
			} else {
				return blueSourceChooser.getSelected();
			}
    }

	public Double getElevatorPosition() {
		return elevatorPositionChooser.getSelected();
	}

    public static void applyTalonConfigs(TalonFX motor, TalonFXConfiguration config) {
		StatusCode status = StatusCode.StatusCodeNotInitialized;
		for (int i = 0; i < 5; ++i) {
			status = motor.getConfigurator().apply(config);
			if (status.isOK())
			break;
		}
		if (!status.isOK()) {
			DataLogManager.log("Erorr:" + motor.getDescription() + " Configuration not applied " + status.toString());
		}
	}

}
