// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

//Imports
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wristsubsystem extends SubsystemBase {
  // Constants
  public static class WristConstants {

    public static final int WRIST_CAN_ID = 30;

    // TODO: Tune and collect the various values here
    // The Wrist Motor Current Limit
    private static final int WRIST_CURRENT_LIMIT = 60;

    // Gearbox reductions
    private static final double WRIST_GEARBOX_RATIO = 48;
    private static final double WRIST_SPROCKET_RATIO = 64/24;
    private static final Angle WRIST_DEGREES_PER_ROTATION = Degrees.of(360 / WRIST_GEARBOX_RATIO / WRIST_SPROCKET_RATIO);

    // Soft Limits
    private static final Angle WRIST_FORWARD_SOFT_LIMIT = Degrees.of(45);
    private static final Angle WRIST_REVERSE_SOFT_LIMIT = Degrees.of(-45);

    // PID constants
    private static final double WRIST_KP = 0.1;
    private static final double WRIST_KI = 0;
    private static final double WRIST_KD = 0;
    

    private static final Angle WRIST_SCORE_POSITION = Degrees.of(0); // TODO: possibly add different scoring positions
    private static final Angle WRIST_INTAKE_POSITION = Degrees.of(0);
  }

  private SparkFlex wristMotor;// Motor Objet
  private SparkClosedLoopController wristPIDController;// Object for the motor's PID
  private RelativeEncoder wristEncoder; // Motor Encoder Object

  private final SparkFlexConfig wristMotorConfig;// Configurator Object

  private double positionMaxOutput;
  private double positionMinOutput;

  public double setAngle;

  public double targetWristAngle = WristConstants.WRIST_REVERSE_SOFT_LIMIT.magnitude();

  /** Creates a new Wristsubsystem. */
  public Wristsubsystem() {
    // Makes the previously defined object a Spark MAX Motor
    wristMotor = new SparkFlex(WristConstants.WRIST_CAN_ID, MotorType.kBrushless);

    wristEncoder = wristMotor.getEncoder();

    // Create the config
    wristMotorConfig = new SparkFlexConfig();
    wristMotorConfig.idleMode(IdleMode.kBrake);// Motor Idle mode when disabled
    wristMotorConfig.smartCurrentLimit(WristConstants.WRIST_CURRENT_LIMIT);// Setting the maximum current limit

    // Setting the soft limits and enabeling them.
    wristMotorConfig.softLimit.forwardSoftLimit(WristConstants.WRIST_FORWARD_SOFT_LIMIT.magnitude());
    wristMotorConfig.softLimit.reverseSoftLimit(WristConstants.WRIST_REVERSE_SOFT_LIMIT.magnitude());
    wristMotorConfig.softLimit.forwardSoftLimitEnabled(true);
    wristMotorConfig.softLimit.reverseSoftLimitEnabled(true);

    wristPIDController = wristMotor.getClosedLoopController();

    wristMotorConfig.encoder.positionConversionFactor(WristConstants.WRIST_DEGREES_PER_ROTATION.magnitude());

    wristMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    // PID

    // Percent of the position of the wrist
    positionMaxOutput = 1;
    positionMinOutput = -1;

    wristMotorConfig.closedLoop.p(WristConstants.WRIST_KP);
    wristMotorConfig.closedLoop.i(WristConstants.WRIST_KI);
    wristMotorConfig.closedLoop.d(WristConstants.WRIST_KD);
    wristMotorConfig.closedLoop.positionWrappingEnabled(false);

    // Setting the range of motion avaliable
    wristMotorConfig.closedLoop.outputRange(positionMinOutput, positionMaxOutput);

    // Apply the configuration to the motor
    wristMotor.configure(wristMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    SmartDashboard.putData("Wrist" , this);
  }

  // Nothing needs to happen here, only when the subsystem is called
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * @brief Sets the position of the Wrist to a desired percentage position with a
   *        PID Control Loop
   * @param position The position percentage to set
   */
  public void setAngle(double angle) {
    wristPIDController.setReference(angle, SparkFlex.ControlType.kPosition, ClosedLoopSlot.kSlot0, 0,
        SparkClosedLoopController.ArbFFUnits.kVoltage); 
  }

  /**
   * @breif Gets the current position of the encoder with the gearbox reduction in
   *        effect
   * @return Curerent position, in degrees
   */
  public double getWristEncoderPosition() {
    return wristEncoder.getPosition();
  }


  public void intiSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Actual Wrist Angle", this::getWristEncoderPosition, null);
  }

}
