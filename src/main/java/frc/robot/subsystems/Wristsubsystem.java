// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

//Imports
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotStates;
import frc.robot.RobotContainer;

public class Wristsubsystem extends SubsystemBase {
  //Constants
  public static class WristConstants{
    
    public static final int WRIST_CAN_ID = 30;
    
    //TODO: Tune and collect the various values here
     //The Wrist Motor Current Limit
     public static final int WRIST_CURRENT_LIMIT = 0;

     //Gearbox reductions
    public static final int WRIST_DEGREES_PER_ROTATION = 0;

    //Soft Limits
    public static final int WRIST_FORWARD_SOFT_LIMIT = 0;
    public static final int WRIST_REVERSE_SOFT_LIMIT = 0;
    
    public static final int WRIST_PID_ID = 0;

    //PID constants
    private static final int WRIST_P = 0;
    private static final int WRIST_I = 0;
    private static final int WRIST_D = 0;
   
    public static final int WRIST_START_POSITION = 0;
    public static final int WRIST_SCORE_POSITION = 0;
    public static final int WRIST_INTAKE_POSITION = 0;
  }

  private SparkMax wristMotor;//Motor Objet
  private SparkClosedLoopController wristPIDController;//Object for the motor's PID
  private RelativeEncoder wristEncoder;//Motor Encoder Object

  private final SparkMaxConfig wristMotorConfig;//Configurator Object
  private final SparkMaxConfig wristPIDControllerSparkMaxConfig;//PID Configurator Object
 
  private double positionMaxOutput;
  private double positionMinOutput;
  
  public double setPoint;

  private double targetWristAngle = WristConstants.WRIST_REVERSE_SOFT_LIMIT;
  
  /** Creates a new Wristsubsystem. */
  public Wristsubsystem() {
    //Makes the previously defined object a Spark MAX Motor
    wristMotor = new SparkMax(WristConstants.WRIST_CAN_ID, MotorType.kBrushless);

    wristEncoder = wristMotor.getEncoder();

    //wristMotor.configure(null, SparkBase.ResetMode.kResetSafeParameters, null);

    //Create the config
    wristMotorConfig = new SparkMaxConfig();
    wristMotorConfig.idleMode(IdleMode.kBrake);//Motor Idle mode when disabled
    wristMotorConfig.smartCurrentLimit(WristConstants.WRIST_CURRENT_LIMIT);//Setting the maximum current limit

    wristPIDControllerSparkMaxConfig = new SparkMaxConfig();

    wristEncoder.setPosition(WristConstants.WRIST_START_POSITION);

    setPoint = WristConstants.WRIST_START_POSITION;

    //Setting the soft limits and enabeling them.
    wristMotorConfig.softLimit.forwardSoftLimit(WristConstants.WRIST_FORWARD_SOFT_LIMIT);
    wristMotorConfig.softLimit.reverseSoftLimit(WristConstants.WRIST_REVERSE_SOFT_LIMIT);
    wristMotorConfig.softLimit.forwardSoftLimitEnabled(true);
    wristMotorConfig.softLimit.reverseSoftLimitEnabled(true);

    wristPIDController = wristMotor.getClosedLoopController();
  
    wristMotorConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

    wristMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
  

    //PID
  
    //Percent of the position of the wrist
    positionMaxOutput = 1;
    positionMinOutput = -1;

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
   * @brief   Sets the position of the Wrist to a desired percentage position with a PID Control Loop
   * @param position  The position percentage to set
   */
  public void setAngle(DoubleSupplier angle){
    targetWristAngle = angle.getAsDouble();
    wristPIDController.setReference(angle.getAsDouble(), SparkMax.ControlType.kPosition, null, 0, SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

 /**
   * @breif   Gets the current position of the encoder with the gearbox reduction in effect
   * @return  Curerent position, in degrees
   */
  public double getWristEncoderPosition(){
    return wristEncoder.getPosition();
  }

  public double getTargetAngle()
  {
    double angle = WristConstants.WRIST_REVERSE_SOFT_LIMIT;
    if(RobotContainer.robotStates.equals(RobotStates.SCORE)){
      angle = WristConstants.WRIST_SCORE_POSITION;
    }
    if(RobotContainer.robotStates.equals(RobotStates.INTAKE)){
      angle = WristConstants.WRIST_INTAKE_POSITION;
    }
        return angle;
  }


   public void intiSendable(SendableBuilder builder)
  {
    super.initSendable(builder);

    builder.addDoubleProperty("Actual Wrist Angle", this::getWristEncoderPosition, null);
  }

  }


