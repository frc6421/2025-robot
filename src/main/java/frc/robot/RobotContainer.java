// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.AlgaeRemovalCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IntakeSequenceCommand;
import frc.robot.commands.ResetAlgaeCommand;
import frc.robot.commands.ScoreFinishCommand;
import frc.robot.commands.ScorePrepCommand;
import frc.robot.commands.ScoreSequenceCommand;
import frc.robot.commands.autoCommands.BlueEAlgaeRBCommand;
import frc.robot.commands.autoCommands.BlueEBARBCommand;
import frc.robot.commands.autoCommands.BlueEDCRBCommand;
import frc.robot.commands.autoCommands.BlueGRBCommand;
import frc.robot.commands.autoCommands.BlueJAlgaeBBCommand;
import frc.robot.commands.autoCommands.BlueJKLBBCommand;
import frc.robot.commands.autoCommands.RedEAlgaeBBCommand;
import frc.robot.commands.autoCommands.RedEDCBBCommand;
import frc.robot.commands.autoCommands.RedHRBCommand;
import frc.robot.commands.autoCommands.RedJABRBCommand;
import frc.robot.commands.autoCommands.RedJAlgaeRBCommand;
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
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;

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
	private CommandXboxController testJoystick;

	private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	private final WristSubsystem wristSubsystem = new WristSubsystem();
	private final ClimbPivotSubsystem climbSubsystem = new ClimbPivotSubsystem();
	private final CageIntakeSubsystem cageIntakeSubsystem = new CageIntakeSubsystem();

	private final ClimbCommand climbCommand = new ClimbCommand(climbSubsystem, cageIntakeSubsystem, wristSubsystem);

	private final SlewRateLimiter xDriveSlew = new SlewRateLimiter(Constants.DriveConstants.DRIVE_SLEW_RATE);
	private final SlewRateLimiter yDriveSlew = new SlewRateLimiter(Constants.DriveConstants.DRIVE_SLEW_RATE);

	private final TestAutoCommand testAuto;
	private final RedJKLRBCommand redJKLRB;
	private final RedEDCBBCommand redEDCBB;
	private final RedHRBCommand redHRB;
	private final RedJAlgaeRBCommand redJAlgaeRB;
	private final RedEAlgaeBBCommand redEAlgaeBB;
	private final RedJABRBCommand redJABRB;
	private final BlueJKLBBCommand blueJKLBB;
	private final BlueEDCRBCommand blueEDCRB;
	private final BlueGRBCommand blueGRB;
	private final BlueJAlgaeBBCommand blueJAlgaeBB;
	private final BlueEAlgaeRBCommand blueEAlgaeRB;
	private final BlueEBARBCommand blueEBARB;
	

	private SendableChooser<Command> redAutoChooser;
	// private SendableChooser<Command> blueAutoChooser;
    private SendableChooser<Pose2d> redPositionChooser;
    private SendableChooser<Pose2d> bluePositionChooser;
    private SendableChooser<Pose2d> redSourceChooser;
    private SendableChooser<Pose2d> blueSourceChooser;
		private SendableChooser<Double> elevatorPositionChooser;

	private final ScoreSequenceCommand scoreSequenceCommand;
	private final IntakeSequenceCommand intakeSequenceCommand;
	private final AlgaeRemovalCommand algaeRemovalCommand;
	private final ResetAlgaeCommand resetAlgaeCommand;
	private final ScorePrepCommand scorePrepCommand;
	private final ScoreFinishCommand scoreFinishCommand;

	Optional<Alliance> alliance = DriverStation.getAlliance();

	public RobotContainer() {

		elevatorPositionChooser = new SendableChooser<>();
				elevatorPositionChooser.setDefaultOption("L1", ElevatorConstants.L1_POSITION.magnitude());
				elevatorPositionChooser.addOption("L2", ElevatorConstants.L2_POSITION.magnitude());
				elevatorPositionChooser.addOption("L3", ElevatorConstants.L3_POSITION.magnitude());
				// elevatorPositionChooser.addOption("L4", ElevatorConstants.L4_POSITION.magnitude());

				scoreSequenceCommand = new ScoreSequenceCommand(elevatorSubsystem, wristSubsystem, intakeSubsystem, () -> getElevatorPosition());
				intakeSequenceCommand = new IntakeSequenceCommand(elevatorSubsystem, wristSubsystem, intakeSubsystem);
				algaeRemovalCommand = new AlgaeRemovalCommand(elevatorSubsystem, wristSubsystem, intakeSubsystem, () -> getElevatorPosition());
				resetAlgaeCommand = new ResetAlgaeCommand(elevatorSubsystem, wristSubsystem, intakeSubsystem);
				scorePrepCommand = new ScorePrepCommand(elevatorSubsystem, wristSubsystem, intakeSubsystem, drivetrain, () -> getElevatorPosition(), () -> getSelectedPoseCommand());
				scoreFinishCommand = new ScoreFinishCommand(elevatorSubsystem, wristSubsystem, intakeSubsystem);


		testAuto = new TestAutoCommand(drivetrain);
		redJKLRB = new RedJKLRBCommand(drivetrain, elevatorSubsystem, wristSubsystem, intakeSubsystem);
		redEDCBB = new RedEDCBBCommand(drivetrain, elevatorSubsystem, wristSubsystem, intakeSubsystem);
		redHRB = new RedHRBCommand(drivetrain, elevatorSubsystem, wristSubsystem, intakeSubsystem);
		redJAlgaeRB = new RedJAlgaeRBCommand(drivetrain, elevatorSubsystem, wristSubsystem, intakeSubsystem);
		redEAlgaeBB = new RedEAlgaeBBCommand(drivetrain, elevatorSubsystem, wristSubsystem, intakeSubsystem);
		redJABRB = new RedJABRBCommand(drivetrain, elevatorSubsystem, wristSubsystem, intakeSubsystem);
		blueJKLBB = new BlueJKLBBCommand(drivetrain, elevatorSubsystem, wristSubsystem, intakeSubsystem);
		blueEDCRB = new BlueEDCRBCommand(drivetrain, elevatorSubsystem, wristSubsystem, intakeSubsystem);
		blueGRB = new BlueGRBCommand(drivetrain, elevatorSubsystem, wristSubsystem, intakeSubsystem);
		blueJAlgaeBB = new BlueJAlgaeBBCommand(drivetrain, elevatorSubsystem, wristSubsystem, intakeSubsystem);
		blueEAlgaeRB = new BlueEAlgaeRBCommand(drivetrain, elevatorSubsystem, wristSubsystem, intakeSubsystem);
		blueEBARB = new BlueEBARBCommand(drivetrain, elevatorSubsystem, wristSubsystem, intakeSubsystem);

		redAutoChooser = new SendableChooser<>();
		//redAutoChooser.addOption("Red JKL RB", redJKLRB);
		//redAutoChooser.addOption("Red EDC BB", redEDCBB);
		redAutoChooser.addOption("Red H", redHRB);
		redAutoChooser.addOption("Red J Algae RB", redJAlgaeRB);
		redAutoChooser.addOption("Red E Algae BB", redEAlgaeBB);
		redAutoChooser.addOption("Red JAB RB", redJABRB);
        //blueAutoChooser.addOption("Blue JKL BB", blueJKLBB);
        //blueAutoChooser.addOption("Blue EDC RB", blueEDCRB);
    redAutoChooser.addOption("Blue G", blueGRB);
		redAutoChooser.addOption("Blue E Algae RB", blueEAlgaeRB);
		redAutoChooser.addOption("Blue J Algae BB", blueJAlgaeBB);
		redAutoChooser.addOption("Blue EBA RB", blueEBARB);

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

		SmartDashboard.putData("Red Auto Chooser", redAutoChooser);
    SmartDashboard.putData("Red Position Chooser", redPositionChooser);
	SmartDashboard.putData("Blue Position Chooser", bluePositionChooser);
    SmartDashboard.putData("Red Source Chooser", redSourceChooser);
	SmartDashboard.putData("Blue Source Chooser", blueSourceChooser);
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





			 joystick.rightBumper().whileTrue(drivetrain.reefAlignCommand(() -> getSelectedPoseCommand()));
		  	joystick.leftBumper().whileTrue(drivetrain.sourceAlignCommand(() -> getSelectedSource()));

				joystick.leftTrigger().onTrue(intakeSequenceCommand);

				//joystick.rightBumper().whileTrue(scorePrepCommand);
				joystick.rightTrigger().onTrue(scoreSequenceCommand);

				joystick.b().onTrue(drivetrain.resetGyro());

				// Manual Overrides
				joystick.start().whileTrue(elevatorSubsystem.stupidStupid());
				joystick.back().whileTrue(wristSubsystem.resetWrist());

				joystick.povLeft().onTrue(drivetrain.nudgeCommand(-1));
				joystick.povRight().onTrue(drivetrain.nudgeCommand(1));

				joystick.a().whileTrue(algaeRemovalCommand);
				joystick.a().onFalse(resetAlgaeCommand);

				//joystick.x().onTrue(climbCommand);

				//joystick.y().whileTrue(climbSubsystem.climbOut()); 
				joystick.y().whileTrue(climbSubsystem.setVoltageCommand(3));
				joystick.y().onFalse(climbSubsystem.setVoltageCommand(0));
				//joystick.x().whileTrue(climbSubsystem.climbIn());
				//joystick.x().whileTrue(climbSubsystem.climbIn());
				joystick.povUp().onTrue(wristSubsystem.setAngle(150));

				if (!DriverStation.isFMSAttached()) {
				testJoystick = new CommandXboxController(3);
				testJoystick.leftTrigger().onTrue(intakeSequenceCommand);
				testJoystick.rightTrigger().onTrue(scoreSequenceCommand);
				testJoystick.x().whileTrue(elevatorSubsystem.setElevatorVoltage(SmartDashboard.getNumber("kG", 0)));
				testJoystick.x().onFalse(elevatorSubsystem.stopElevator());
				}


		drivetrain.registerTelemetry(logger::telemeterize);
	}

    public Command getAutonomousCommand() {
		return redAutoChooser.getSelected();
	}

    public Pose2d getSelectedPoseCommand() {
			alliance = DriverStation.getAlliance();
		if (alliance.isPresent()) {
			if (alliance.get() == Alliance.Red) {
				System.out.println("red pos");
				return redPositionChooser.getSelected();
			} else {
				System.out.println("blue pos");
				return bluePositionChooser.getSelected();
			}
		} else {
			System.out.println("no alliance red pos");
			return redPositionChooser.getSelected();
		}
    }

    public Pose2d getSelectedSource() {
			alliance = DriverStation.getAlliance();
		if (alliance.isPresent()) {
			if (alliance.get() == Alliance.Red) {
				System.out.println("red source");
				return redSourceChooser.getSelected();
			} else {
				System.out.println("blue source");
				return blueSourceChooser.getSelected();
			}
		} else {
			System.out.println("no alliance red source");
			return redSourceChooser.getSelected();
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
