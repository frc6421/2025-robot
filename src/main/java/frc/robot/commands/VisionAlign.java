// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.WarriorCamera;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionAlign extends Command {
  /** Creates a new VisionAlign. */
  private WarriorCamera warriorCamera;
  private CommandSwerveDrivetrain driveTrain;
  private SwerveRequest.FieldCentricFacingAngle visionDriveRequest;
  private double allowableError = 1.0;
  private double allowableVelocityError = 10.0;
  private double allowableRotationalError = 0.035;
  private Pose2d currentPose;

  private Rotation2d targetRotation;
  private Rotation2d adjustRotation;

  private MedianFilter medianFilter = new MedianFilter(5);
  private Optional<DriverStation.Alliance> allianceColor;
  
  private double maxSpeed = 4.7;
  private double xSpeed = 0;
  private double ySpeed = 0;

  private double currentX = 0;
  private double currentY = 0;

  private double controllerP = .1;
  private double controllerD = 0.0016;
  private double rotationalP = 5.0;

  private final PIDController xController = new PIDController(controllerP, 0, controllerD);
  private final PIDController yController = new PIDController(controllerP, 0, controllerD);

  private boolean isRunning = false;


  public VisionAlign(WarriorCamera warriorCamera, CommandSwerveDrivetrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.warriorCamera = warriorCamera;
    this.driveTrain = driveTrain;
    visionDriveRequest = new SwerveRequest.FieldCentricFacingAngle()
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    visionDriveRequest.HeadingController.setPID(rotationalP, 0, 0);
    visionDriveRequest.HeadingController.setTolerance(allowableRotationalError);
    visionDriveRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(driveTrain);
  }

   /** Drives to the specified pose under full software control. */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isRunning = true;
    currentPose = driveTrain.getState().Pose;
    xController.reset();
    yController.reset();
    medianFilter.reset();

    xController.setTolerance(allowableError, allowableVelocityError); // sourced from TunerConstants
    yController.setTolerance(allowableError, allowableVelocityError);

    allianceColor = DriverStation.getAlliance();

    if (!allianceColor.isPresent())  {
      isRunning = false;
      System.out.println("AmpVisionCommand canceled - No alliance color present");
    }

    // Set setpoint to turn the robot to the correct angle (degrees to work with the
    // drive request)
    if (warriorCamera.getTagFieldLayout().getTagPose(warriorCamera.getBestTagId()).isPresent()) {

      targetRotation = warriorCamera.getTagFieldLayout().getTagPose(warriorCamera.getBestTagId()).get().getRotation().toRotation2d();

    } else {

      targetRotation = currentPose.getRotation();
      
      isRunning = false;
      System.out.println("AmpVisionCommand canceled - No AprilTag pose present");

    }
    SmartDashboard.putData("VisionAlign", this);
    SmartDashboard.putNumber("VisionAlign/Target X (Yaw)", xController.getSetpoint());
    SmartDashboard.putNumber("VisionAlign/Target Y (Pitch)", yController.getSetpoint());
    SmartDashboard.putNumber("VisionAlign/Target Rotation (Radians)", visionDriveRequest.HeadingController.getSetpoint());

    SmartDashboard.putNumber("VisionAlign/Current X (Yaw)", currentX);
    SmartDashboard.putNumber("VisionAlign/Current Y (Pitch)", currentY);
    SmartDashboard.putNumber("VisionAlign/Current Rotation (Radians)", driveTrain.getPigeon2().getRotation2d().getRadians());
    SmartDashboard.putNumber("VisionAlign/Current Pose Rotation", currentPose.getRotation().getRadians());

    SmartDashboard.putNumber("VisionAlign/X Speed", xSpeed);
    SmartDashboard.putNumber("VisionAlign/Y Speed", ySpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentX = medianFilter.calculate(warriorCamera.getYaw());
    currentY = medianFilter.calculate(warriorCamera.getPitch());

      // Set setpoint to center the robot on the amp in the x direction
      //xController.setSetpoint(VisionConstants.AMP_FAR_YAW_ANGLE);

      // Set setpoint to drive the robot up against the amp
      //yController.setSetpoint(VisionConstants.AMP_FAR_PITCH_ANGLE);

    // Ends command if no AprilTag is detected in the camera frame
    // Camera methods return 180.0 if the target tag ID is not detected
    if (currentX == 180.0 || currentY == 180.0) {

      isRunning = false;
      System.out.println("AmpVisionCommand canceled - No AprilTag detected");

      adjustRotation = currentPose.getRotation();

    } else {

      if (xController.atSetpoint()) {

        xSpeed = 0.0;

      } else {

        xSpeed = MathUtil.clamp(xController.calculate(currentX), -maxSpeed, maxSpeed);

      }

      if (yController.atSetpoint()) {

        ySpeed = 0.0;

      } else {

        ySpeed = MathUtil.clamp(yController.calculate(currentY), -maxSpeed, maxSpeed);

      }

      //adjustRotation = targetRotation.minus(new Rotation2d(Units.degreesToRadians(((currentX + VisionConstants.AMP_CLOSE_YAW_ANGLE) / 2))));
      
    }

    if(adjustRotation.equals(null)) {

      System.out.println("Null angle");
      isRunning = false;

    } else {

      driveTrain.setControl(
      visionDriveRequest.withVelocityX(-xSpeed)
          .withVelocityY(ySpeed)
          .withTargetDirection(adjustRotation));

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xController.atSetpoint() && yController.atSetpoint() && visionDriveRequest.HeadingController.atSetpoint())
      || !isRunning;
  }

}
