// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
//Imports
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
  //Constants
  public static class WristConstants{
    
    public static final int WRIST_CAN_ID = 30;
    //PID Slot 0
    //TODO: Tune and collect the various values here
    public static final int WRIST_PID_ID = 0;
    //The Wrist Motor Current Limit
    public static final int WRIST_CURRENT_LIMIT = 0;
    //Zero position of the Writs, or the rest position
    public static final int WRIST_START_POSITION = 0;
    //Gearbox reductions
    public static final int WRIST_DEGREES_PER_ROTATION = 0;
    //Soft Limits
    public static final int WRIST_OUT_SOFT_LIMIT = 0;
    public static final int WRIST_IN_SOFT_LIMIT = 0;
    //PID constants
    private static final int WRIST_P = 0;
    private static final int WRIST_I = 0;
    private static final int WRIST_D = 0;
    //Gravity Feed Forward
    private static final int MAX_WRIST_GRAVITY_FF = 0;
  }

  private SparkMax wristMotor;//Motor Objet
  private SparkClosedLoopController wristPIDController;//Object for the motor's PID
  private RelativeEncoder wristEncoder;//Motor Encoder Object

  private final SparkMaxConfig wristMotorConfig;//Configurator Object
  private final SparkMaxConfig wristPIDControllerSparkMaxConfig;//PID Configurator Object
  //Relative Positions of the motor when considering the gearbox 
  private double positionMaxOutput;
  private double positionMinOutput;
  //Changing Gravity Feed Forward for various angles
  private double wristDynamicFF;
  //Angle to see the arm to
  public double setPoint;
  private ClosedLoopSlot slot;//PID Slot

  /** Creates a new Wristsubsystem. */
  public Wristsubsystem() {
    //Makes the previously defined object a Spark MAX Motor
    wristMotor = new SparkMax(WristConstants.WRIST_CAN_ID, MotorType.kBrushless);
    //wristMotor.configure(null, SparkBase.ResetMode.kResetSafeParameters, null);
    //Create the config
    wristMotorConfig = new SparkMaxConfig();
    wristMotorConfig.idleMode(IdleMode.kBrake);//Motor Idle mode when disabled
    wristMotorConfig.smartCurrentLimit(WristConstants.WRIST_CURRENT_LIMIT);//Setting the maximum current limit
    //Setting the soft limits and enabeling them.
    wristMotorConfig.softLimit.forwardSoftLimit(WristConstants.WRIST_IN_SOFT_LIMIT);
    wristMotorConfig.softLimit.reverseSoftLimit(WristConstants.WRIST_OUT_SOFT_LIMIT);
    wristMotorConfig.softLimit.forwardSoftLimitEnabled(true);
    wristMotorConfig.softLimit.reverseSoftLimitEnabled(true);
    //Setting the encoder to be the built-in encoder on the motor, and "zeros" it to the start position
    wristEncoder = wristMotor.getEncoder();
    wristEncoder.setPosition(WristConstants.WRIST_START_POSITION);

    //PID
    wristPIDControllerSparkMaxConfig = new SparkMaxConfig();
    //Percent of the position of the wrist
    positionMaxOutput = 1;
    positionMinOutput = -1;
    //Applying the PID's
    wristPIDControllerSparkMaxConfig.closedLoop.p(WristConstants.WRIST_P);
    wristPIDControllerSparkMaxConfig.closedLoop.i(WristConstants.WRIST_I);
    wristPIDControllerSparkMaxConfig.closedLoop.d(WristConstants.WRIST_D);
    //Setting the range of motion avaliable
    wristPIDControllerSparkMaxConfig.closedLoop.outputRange(positionMinOutput, positionMaxOutput);
    //Apply the configuration to the motor
    wristMotor.configure(wristMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
  }

  //Nothing needs to happen here, only when the subsystem is called
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * @breif   Sets the position of the Wrist to 0 with a PID Control Loop
   */
  public void setPosition(){
    wristPIDController.setReference(0, SparkMax.ControlType.kPosition);
  }

  /**
   * @brief   Sets the position of the Wrist to a desired percentage position with a PID Control Loop
   * @param position  The position percentage to set
   */
  public void setPosition(double position){
    wristPIDController.setReference(position, ControlType.kPosition);
  }

  /**
   * @breif   Sets the angle while taking into account the force of gravity for any angle
   * @param angle   The angle to set to, in degrees
   */
  public void setWristAngleWithGrav(double angle){
    setGravityOffset();
    wristPIDController.setReference(angle, ControlType.kCurrent, null, wristDynamicFF, SparkClosedLoopController.ArbFFUnits.kPercentOut);
  }

  /**
   * @breif   Sets the angle of the Wrist taking into account the Feed Forward of the Wrist
   * @param angle   The angle to set to, in degrees
   * @param newFF   The new Feed Forward to set
   */
  public void setWristAngleAndFF(double angle, double newFF){
    wristPIDController.setReference(setPoint, ControlType.kPosition, null, wristDynamicFF, SparkClosedLoopController.ArbFFUnits.kPercentOut);
  }

  /**
   * @breif   Calculates a new Gravity Offset 
   */
  public void setGravityOffset(){
    wristDynamicFF = (WristConstants.MAX_WRIST_GRAVITY_FF * Math.cos(Math.toRadians(getWristDegreePosition())));
  }

  /**
   * @breif   Gives the Dynamic Feed Forward of the Wrist
   * @return  The current Dynamic Feed Forward
   */
  public double getFeedForward(){
    return wristDynamicFF;
  }

  /**
   * @breif   Updates the current set point of the Wrist
   * @param setPoint  The set point to set to
   */
  public void setSetPoint(double setPoint) {
    this.setPoint = setPoint;
  }

  /**
   * @breif   Gets the current position of the encoder with the gearbox reduction in effect
   * @return  Curerent position, in degrees
   */
  public double getWristDegreePosition(){
    return wristEncoder.getPosition();
  }

  /**
   * @breif   Gives the current set point of the Wrist
   * @return  The current Set Point
   */
  public double getSetPoint() {
    return setPoint;
  }

  /**
   * @breif   Resets the encoder to the Wrist Start
   */
  public void resetEncoderPosition(){
    wristEncoder.setPosition(WristConstants.WRIST_START_POSITION);
  }

  }


