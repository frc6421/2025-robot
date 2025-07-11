// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import javax.xml.crypto.Data;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ClimbPivotSubsystem extends SubsystemBase {

  private final TalonFX leftPivotMotor;
  private final TalonFX rightPivotMotor;
  private final DigitalInput pivotLimitSwitch;

  private final TalonFXConfiguration leftPivotMotorConfig;
  private final TalonFXConfiguration rightPivotMotorConfig;

  private final VoltageOut voltageRequest;

  private StatusCode status = StatusCode.StatusCodeNotInitialized;
  private boolean isClimberOut = false;

  public static class ClimbPivotConstants {

    private static final int LEFT_PIVOT_MOTOR_ID = 50;
    private static final int RIGHT_PIVOT_MOTOR_ID = 51;
    private static final int PIVOT_LIMIT_SWITCH_PORT = 0;

    private static final double CLIMB_MOTOR_GEAR_RATIO = 5 * 5;
    private static final double CLIMB_TOTAL_RATIO = CLIMB_MOTOR_GEAR_RATIO;
    private static final double CLIMB_DEGREES_PER_MOTOR_ROTATION = 360 / CLIMB_TOTAL_RATIO;
    //private static final double CLIMB_DEGREES_PER_MOTOR_ROTATION = 87.5 / 236.85; // TODO: check why this is like this

    private static final Current PIVOT_CURRENT_LIMIT = Amps.of(250); // TODO: Update Numbers

    private static final CurrentLimitsConfigs PIVOT_CURRENT_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(PIVOT_CURRENT_LIMIT)
        .withSupplyCurrentLimit(PIVOT_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true);

    //Soft limits
    /** In rotations */
    public static final double PIVOT_FORWARD_SOFT_LIMIT = 10000 / CLIMB_DEGREES_PER_MOTOR_ROTATION; // TODO: Update Numbers
    /** In rotations */
    public static final double PIVOT_REVERSE_SOFT_LIMIT = -10000 / CLIMB_DEGREES_PER_MOTOR_ROTATION; // TODO: Update Numbers
    /** In rotations */
    public static final double PIVOT_OUT_POSITION = -646.0; //here
    /** In rotations */
    public static final double PIVOT_IN_POSITION = -500.0;

    private static final SoftwareLimitSwitchConfigs PIVOT_SOFT_LIMIT_CONFIGS = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitThreshold(ClimbPivotConstants.PIVOT_FORWARD_SOFT_LIMIT)
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(ClimbPivotConstants.PIVOT_REVERSE_SOFT_LIMIT)
        .withReverseSoftLimitEnable(true);

    private static final MotorOutputConfigs LEFT_PIVOT_MOTOR_CONFIGS = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);

    private static final MotorOutputConfigs RIGHT_PIVOT_MOTOR_CONFIGS = new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);

  }

  /** Creates a new ClimbSubsystem. */
  public ClimbPivotSubsystem() {
    leftPivotMotor = new TalonFX(ClimbPivotConstants.LEFT_PIVOT_MOTOR_ID);
    rightPivotMotor = new TalonFX(ClimbPivotConstants.RIGHT_PIVOT_MOTOR_ID);
    pivotLimitSwitch = new DigitalInput(ClimbPivotConstants.PIVOT_LIMIT_SWITCH_PORT);

    // Set controller to factory default
    RobotContainer.applyTalonConfigs(leftPivotMotor, new TalonFXConfiguration());
    RobotContainer.applyTalonConfigs(rightPivotMotor, new TalonFXConfiguration());

    leftPivotMotorConfig = new TalonFXConfiguration()
        .withCurrentLimits(ClimbPivotConstants.PIVOT_CURRENT_CONFIGS)
        .withMotorOutput(ClimbPivotConstants.LEFT_PIVOT_MOTOR_CONFIGS)
        .withSoftwareLimitSwitch(ClimbPivotConstants.PIVOT_SOFT_LIMIT_CONFIGS);

    rightPivotMotorConfig = new TalonFXConfiguration()
        .withCurrentLimits(ClimbPivotConstants.PIVOT_CURRENT_CONFIGS)
        .withMotorOutput(ClimbPivotConstants.RIGHT_PIVOT_MOTOR_CONFIGS)
        .withSoftwareLimitSwitch(ClimbPivotConstants.PIVOT_SOFT_LIMIT_CONFIGS);

    RobotContainer.applyTalonConfigs(leftPivotMotor, leftPivotMotorConfig);
    RobotContainer.applyTalonConfigs(rightPivotMotor, rightPivotMotorConfig);

    // // Set up follower
    // Follower follower = new Follower(rightPivotMotor.getDeviceID(), true);
    // status = StatusCode.StatusCodeNotInitialized;
    // for (int i = 0; i < 5; i++) {
    //   status = leftPivotMotor.setControl(follower);
    //   if (status.isOK()){
    //     break;
    //   }
    // }
    // if (!status.isOK()) {
    //   DataLogManager.log("Follower not applied " + leftPivotMotor + " Statis code " + status.toString());
    // };

    voltageRequest = new VoltageOut(0);

    // Set initial motor position in rotations with error checking.
    setPosition(leftPivotMotor,
        ClimbPivotConstants.PIVOT_REVERSE_SOFT_LIMIT);
    setPosition(rightPivotMotor,
        ClimbPivotConstants.PIVOT_REVERSE_SOFT_LIMIT);

    SmartDashboard.putData("Climb Pivots", this);
  }

  @Override
  public void periodic() {
  }

  // Set methods \\

  public Command setVoltageCommand(double voltage) {
    return run(() -> {
          rightPivotMotor.setControl(voltageRequest.withOutput(voltage));
          leftPivotMotor.setControl(voltageRequest.withOutput(voltage));
    }
  );//.finallyDo(() -> stopPivotMotors());
}

  public void setVoltage(double voltage) {
    rightPivotMotor.setControl(voltageRequest.withOutput(voltage));
    leftPivotMotor.setControl(voltageRequest.withOutput(voltage));
  }

  public void stopPivotMotors() {
     leftPivotMotor.stopMotor();
    rightPivotMotor.stopMotor();
  }

  /**
   * Retry set postion up to 5 times, report if failure
   * 
   * @param motor    TalonFX
   * @param position postion in rotations
   */
  private void setPosition(TalonFX motor, double position) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = motor.setPosition(position);
      if (status.isOK()) {
        break;
      }
    }
    if (!status.isOK()) {
      DataLogManager.log("Set Postion Error " + motor.getDescription() + " Status code: " + status.toString());
    }
  }

  // Get methods \\

  private double getPivotAngle(TalonFX motor) {
    return ClimbPivotConstants.CLIMB_DEGREES_PER_MOTOR_ROTATION * motor.getPosition().getValueAsDouble();
  }

  public boolean isInPosition() {
    return rightPivotMotor.getPosition().getValueAsDouble() >= ClimbPivotConstants.PIVOT_IN_POSITION;
  }

  public boolean isOutPosition() {
    return rightPivotMotor.getPosition().getValueAsDouble() >= ClimbPivotConstants.PIVOT_OUT_POSITION;
  }

  /**
   * If limit switch is interrupted returns true, otherwise false
   * @return true/false
   */
  public boolean isLimit() {
    return !pivotLimitSwitch.get();
  }

  public Command climbOut() {
    return setVoltageCommand(1)
     .until(() -> isOutPosition())
     .andThen(() -> stopPivotMotors());
  }

  public Command climbIn() {
    return setVoltageCommand(1)
    .until(() -> isLimit())
    .andThen(() -> stopPivotMotors());
    }

  @Override
  public void initSendable(SendableBuilder builder) {

    super.initSendable(builder);
    builder.addDoubleProperty("Left Pivot Angle", () -> getPivotAngle(leftPivotMotor), null);
    builder.addDoubleProperty("Right Pivot Angle", () -> getPivotAngle(rightPivotMotor), null);
    builder.addDoubleProperty("Left Motor Rotations", () -> leftPivotMotor.getPosition().getValueAsDouble(), null);
    builder.addDoubleProperty("Right Motor Rotations", () -> rightPivotMotor.getPosition().getValueAsDouble(), null);
    builder.addDoubleProperty("Left Pivot Current", () -> leftPivotMotor.getStatorCurrent().getValueAsDouble(), null);
    builder.addDoubleProperty("Right Pivot Current", () -> rightPivotMotor.getStatorCurrent().getValueAsDouble(), null);
    builder.addBooleanProperty("Limit Switch", () -> isLimit(), null);
    builder.addBooleanProperty("Is Climber Out", () -> isOutPosition(), null);

  }
}