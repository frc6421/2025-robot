// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.AbsoluteEncoder;
//Imports
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
  // Constants
  public static class WristConstants {

    public static final int WRIST_CAN_ID = 30;

    // The Wrist Motor Current Limit
    private static final int WRIST_CURRENT_LIMIT = 80;

    // Gearbox reductions
    private static final double WRIST_GEARBOX_RATIO = 16.0;
    private static final double WRIST_SPROCKET_RATIO = 78.0/30.0;
    private static final Angle WRIST_DEGREES_PER_ROTATION = Degrees.of(360.0 / WRIST_GEARBOX_RATIO / WRIST_SPROCKET_RATIO);

    private static final boolean WRIST_ABS_INVERTED = false;
    private static final double WRIST_ABS_OFFSET = 168.5 / 360.0;

    // Soft Limits
    public static final Angle WRIST_FORWARD_SOFT_LIMIT = Degrees.of(270);
    public static final Angle WRIST_REVERSE_SOFT_LIMIT = Degrees.of(16); // TODO needs to chnage with new numbers

    //TODO: GET THE NEW CONSTANTS
    //PID Constants
    public static final double WRIST_KP = 0.0080;
    public static final double WRIST_KI = 0.0;
    public static final double WRIST_KD = 0.0;
    public static final double WRIST_KS = 0.01;
    public static final double WRIST_KG = (0.3759 - WRIST_KS);

    //MAXMotion constant
    private static final double WRIST_ALLOWABLE_ERROR = 0.5;
    private static final double WRIST_MAX_ACCELERATION = 120000;
    private static final double WRIST_MAX_VELOCITY = 60000;
    private static final double POSITION_MAX_OUTPUT = 1;
    private static final double POSITION_MIN_OUTPUT = -1;
    

    public static final Angle WRIST_SCORE_POSITION = Degrees.of(211); // L2 & L3
    public static final Angle WRIST_SCORE_POSITION_4 = Degrees.of(225); 
    public static final Angle WRIST_ALGAE_POSITION = Degrees.of(25);
    public static final Angle WRIST_INTAKE_POSITION = Degrees.of(20);
    public static final Angle WRIST_RESTING_POSITION = Degrees.of(110);
    public static final Angle WRIST_STARTING_CONFIG = Degrees.of(269.3);
  }

  private final SparkFlex wristMotor;// Motor Objet
  private final SparkClosedLoopController wristPIDController;// Object for the motor's PID
  //private final RelativeEncoder wristEncoder; // Motor Encoder Object
  private final SparkAbsoluteEncoder wristAbsoluteEncoder; // Motor Encoder Object
  private final SparkFlexConfig wristMotorConfig;// Configurator Object
  private final AbsoluteEncoderConfig wristAbsoluteConfig;

  public double setAngle;

  public double targetWristAngle = WristConstants.WRIST_STARTING_CONFIG.magnitude();

  /** Creates a new Wristsubsystem. */
  public WristSubsystem() {
    // Makes the previously defined object a Spark MAX Motor
    wristMotor = new SparkFlex(WristConstants.WRIST_CAN_ID, MotorType.kBrushless);

    //wristEncoder = wristMotor.getEncoder();
    wristAbsoluteEncoder = wristMotor.getAbsoluteEncoder();

    wristAbsoluteConfig = new AbsoluteEncoderConfig();
    wristAbsoluteConfig.inverted(WristConstants.WRIST_ABS_INVERTED);
    wristAbsoluteConfig.positionConversionFactor(360);
    wristAbsoluteConfig.zeroOffset(WristConstants.WRIST_ABS_OFFSET);

    // Create the config
    wristMotorConfig = new SparkFlexConfig();
    wristMotorConfig.idleMode(IdleMode.kCoast);// Motor Idle mode when disabled
    wristMotorConfig.smartCurrentLimit(WristConstants.WRIST_CURRENT_LIMIT);// Setting the maximum current limit

    wristMotorConfig.apply(wristAbsoluteConfig);

    // Setting the soft limits and enabeling them.
    wristMotorConfig.softLimit.forwardSoftLimit(WristConstants.WRIST_FORWARD_SOFT_LIMIT.magnitude());
    wristMotorConfig.softLimit.reverseSoftLimit(WristConstants.WRIST_REVERSE_SOFT_LIMIT.magnitude());
    wristMotorConfig.softLimit.forwardSoftLimitEnabled(true);
    wristMotorConfig.softLimit.reverseSoftLimitEnabled(true);

    wristPIDController = wristMotor.getClosedLoopController();

    wristMotorConfig.encoder.positionConversionFactor(WristConstants.WRIST_DEGREES_PER_ROTATION.magnitude());
    //wristMotorConfig.encoder.velocityConversionFactor(WristConstants.WRIST_DEGREES_PER_ROTATION.magnitude());

    wristMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    // PID

    wristMotorConfig.closedLoop.p(WristConstants.WRIST_KP);
    wristMotorConfig.closedLoop.i(WristConstants.WRIST_KI);
    wristMotorConfig.closedLoop.d(WristConstants.WRIST_KD);
    wristMotorConfig.closedLoop.positionWrappingEnabled(false);
    wristMotorConfig.closedLoop.maxMotion
        .maxVelocity(WristConstants.WRIST_MAX_VELOCITY)
        .maxAcceleration(WristConstants.WRIST_MAX_ACCELERATION)
        .allowedClosedLoopError(WristConstants.WRIST_ALLOWABLE_ERROR);
    wristMotorConfig.inverted(true);

    // Setting the range of motion avaliable
    wristMotorConfig.closedLoop.outputRange(WristConstants.POSITION_MIN_OUTPUT, WristConstants.POSITION_MAX_OUTPUT);

    // Apply the configuration to the motor
    wristMotor.configure(wristMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    getWristAbsoluteEncoderPosition();
    //wristEncoder.setPosition(getWristAbsoluteEncoderPosition());
    

    SmartDashboard.putData("Wrist" , this);
  }

  // Nothing needs to happen here, only when the subsystem is called
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  public Command setAngle(double angle) {
    //What was used: (kS + kG) * cos(wristPosition)
    // double angleError = 2;
    return this.runOnce(() -> wristPIDController.setReference(angle, SparkBase.ControlType.kPosition));
    // .until(() -> Math.abs(getWristEncoderPosition() - (angle)) < angleError);
  }

  /**
   * Sets the position of the Wrist to a desired position with a PID Control Loop and Feed Forward
   * @param angle The angle to set to
   * @param volts The voltage, calculated from the WristCommand
   * @return  The Command to run
   */
  // public Command setAngle(double angle, double volts){
  //   return this.runOnce(() -> wristPIDController.setReference(angle, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, volts, ArbFFUnits.kVoltage));
  // }

  public void setAngle(double angle, double volts){
    wristPIDController.setReference(angle, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, volts, ArbFFUnits.kVoltage);
  }

  public void nudgeWristUp() {
    setAngle(getWristAbsoluteEncoderPosition() - 3.0, -0.6);
  }

  public void nudgeWristDown() {
    setAngle(getWristAbsoluteEncoderPosition() + 2.0, 0.4);
  }

  public Command resetWrist() {
    double voltage = 0.5;
    return runOnce(() -> {
      wristMotorConfig.softLimit.forwardSoftLimitEnabled(false);
      wristMotorConfig.softLimit.reverseSoftLimitEnabled(false);
      wristMotor.configure(wristMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }).andThen(setWristVoltage(voltage))

    .finallyDo(() -> {
      stopWrist();
      //wristEncoder.setPosition(WristConstants.WRIST_REVERSE_SOFT_LIMIT.magnitude());
      wristMotorConfig.softLimit.forwardSoftLimitEnabled(true);
      wristMotorConfig.softLimit.reverseSoftLimitEnabled(true);
      wristMotor.configure(wristMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    });
  }

  public Command setWristVoltage(double voltage) {
    return run(() -> wristMotor.setVoltage(voltage));
  }

  /**
   * @breif Gets the current position of the encoder with the gearbox reduction in
   *        effect
   * @return Curerent position, in degrees
   */
  // public double getWristEncoderPosition() {
  //   // System.out.println(wristEncoder.getPosition());
  //   return wristEncoder.getPosition();
  // }
  /**
   * Degrees
   */
  public double getWristAbsoluteEncoderPosition() {
    return wristAbsoluteEncoder.getPosition();
  }

  // public double getWristVelocity() {
  //   return wristAbsoluteEncoder.getVelocity();
  // }

  private double getWristCurrent() {
    return wristMotor.getOutputCurrent();
  }

  private double getWristVoltage() {
    return wristMotor.getAppliedOutput();
  }

  public Command stopWrist() {
    return runOnce(() -> wristMotor.stopMotor());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    //builder.addDoubleProperty("Wrist Angle", this::getWristEncoderPosition, null);
    builder.addDoubleProperty("Wrist Absolute Angle", this::getWristAbsoluteEncoderPosition, null);
    //builder.addDoubleProperty("Wrist Velocity", this::getWristVelocity, null);
    builder.addDoubleProperty("Wrist Current", this::getWristCurrent, null);
    builder.addDoubleProperty("Wrist Voltage", this::getWristVoltage, null);
    
  }

}
