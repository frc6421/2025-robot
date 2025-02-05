// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.autoCommands.BlueEDCRBCommand;
import frc.robot.commands.autoCommands.BlueGRBCommand;
import frc.robot.commands.autoCommands.BlueJKLBBCommand;
import frc.robot.commands.autoCommands.RedEDCBBCommand;
import frc.robot.commands.autoCommands.RedHRBCommand;
import frc.robot.commands.autoCommands.RedJKLRBCommand;
import frc.robot.commands.autoCommands.TestAutoCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

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

    public RobotContainer() {
        configureBindings();

        testAuto = new TestAutoCommand(drivetrain);
        redJKLRB = new RedJKLRBCommand(drivetrain);
        redEDCBB = new RedEDCBBCommand(drivetrain);
        redHRB = new RedHRBCommand(drivetrain);
        blueJKLBB = new BlueJKLBBCommand(drivetrain);
        blueEDCRB = new BlueEDCRBCommand(drivetrain);
        blueGRB = new BlueGRBCommand(drivetrain);

        autoChooser = new SendableChooser<>();
        autoChooser.addOption("Auto Test", testAuto);
        autoChooser.addOption("Red JKL RB", redJKLRB);
        autoChooser.addOption("Red EDC BB", redEDCBB);
        autoChooser.addOption("Red H RB", redHRB);
        autoChooser.addOption("Blue JKL BB", blueJKLBB);
        autoChooser.addOption("Blue EDC RB", blueEDCRB);
        autoChooser.addOption("Blue G RB", blueGRB);

        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Gyro", drivetrain.getPigeon2());
    }

    private void configureBindings() {

        joystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        joystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(xDriveSlew.calculate(-joystick.getLeftY() * MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(yDriveSlew.calculate(-joystick.getLeftX() * MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.rightBumper().whileTrue(drivetrain.reefAlignCommand(Constants.TrajectoryConstants.RED_F));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
