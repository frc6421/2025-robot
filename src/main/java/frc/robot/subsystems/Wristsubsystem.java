// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wristsubsystem extends SubsystemBase {

  public static class WristConstants{
    
    public static final int WRIST_CAN_ID = 30;

    public static final int WRIST_PID_ID = 0;

    public static final int WRIST_CURRENT_LIMIT = 0;

    public static final int WRIST_START_POSITION = 0;

    public static final int WRIST_DEGREES_PER_ROTATION = 0;

    public static final int WRIST_OUT_SOFT_LIMIT = 0;
    public static final int WRIST_IN_SOFT_LIMIT = 0;

    private static final int WRIST_P = 0;
    private static final int WRIST_I = 0;
    private static final int WRIST_D = 0;

    private static final int MAX_WRIST_GRAVITY_FF = 0;
  }

  private SparkMax wristMotor;
  private SparkClosedLoopController wristPIDController; 
  private RelativeEncoder wristEncoder;

  private final SparkMaxConfig wristMotorConfig;
  private final SparkMaxConfig wristPIDControllerSparkMaxConfig;

  private double positionMaxOutput;
  private double positionMinOutput;

  private double wristDynamicFF;

  public double setPoint;
  private ClosedLoopSlot slot;

  /** Creates a new Wristsubsystem. */
  public Wristsubsystem() {
    wristMotor = new SparkMax(WristConstants.WRIST_CAN_ID, MotorType.kBrushless);
    //wristMotor.configure(null, SparkBase.ResetMode.kResetSafeParameters, null);

    wristMotorConfig = new SparkMaxConfig();
    wristMotorConfig.idleMode(IdleMode.kBrake);
    wristMotorConfig.smartCurrentLimit(WristConstants.WRIST_CURRENT_LIMIT);

    wristMotorConfig.softLimit.forwardSoftLimit(WristConstants.WRIST_IN_SOFT_LIMIT);
    wristMotorConfig.softLimit.reverseSoftLimit(WristConstants.WRIST_OUT_SOFT_LIMIT);
    wristMotorConfig.softLimit.forwardSoftLimitEnabled(true);
    wristMotorConfig.softLimit.reverseSoftLimitEnabled(true);

    wristEncoder = wristMotor.getEncoder();
    wristEncoder.setPosition(WristConstants.WRIST_START_POSITION);

    //PID
    wristPIDControllerSparkMaxConfig = new SparkMaxConfig();
    
    positionMaxOutput = 1;
    positionMinOutput = -1;

    wristPIDControllerSparkMaxConfig.closedLoop.p(WristConstants.WRIST_P);
    wristPIDControllerSparkMaxConfig.closedLoop.i(WristConstants.WRIST_I);
    wristPIDControllerSparkMaxConfig.closedLoop.d(WristConstants.WRIST_D);
  
    wristPIDControllerSparkMaxConfig.closedLoop.outputRange(positionMinOutput, positionMaxOutput);

    wristMotor.configure(wristMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPosition()
  {
    wristPIDController.setReference(0, SparkMax.ControlType.kPosition);
  }

  public void setPosition(double position)
  {
    wristPIDController.setReference(position, ControlType.kPosition);
  }

  public void setWristAngleWithGrav(double angle)
  {
    setGravityOffset();
    wristPIDController.setReference(angle, ControlType.kCurrent, null, wristDynamicFF, SparkClosedLoopController.ArbFFUnits.kPercentOut);
  }

  public void setWristAngleAndFF(double angle, double newFF)
  {
    wristPIDController.setReference(setPoint, ControlType.kPosition, null, wristDynamicFF, SparkClosedLoopController.ArbFFUnits.kPercentOut);
  }

  public void setGravityOffset()
  {
    wristDynamicFF = (WristConstants.MAX_WRIST_GRAVITY_FF * Math.cos(Math.toRadians(getWristDegreePosition())));
  }

  public double getFeedForward()
  {
    return wristDynamicFF;
  }

  public void setSetPoint(double setPoint) {
    this.setPoint = setPoint;
  }

  public double getWristDegreePosition()
  {
    return wristEncoder.getPosition();
  }

  public double getSetPoint() {
    return setPoint;
  }

  public void resetEncoderPosition(){
    wristEncoder.setPosition(WristConstants.WRIST_START_POSITION);
  }

  }


