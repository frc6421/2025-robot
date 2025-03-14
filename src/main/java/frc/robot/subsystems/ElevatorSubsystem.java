// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class ElevatorSubsystem extends SubsystemBase {
  // Creating the Motor Controller objects
  private final TalonFX elevatorLeftMotor;
  private final TalonFX elevatorRightMotor;

  // Creating the Motor Controller Configuration objects
  private final TalonFXConfiguration elevatorLeftConfig;
  private final TalonFXConfiguration elevatorRightConfig;
  private final Follower elevatorFollower;

  private MotionMagicVoltage elevatorRequest;// Voltage to maintian the set point
  private VoltageOut voltageRequest;

  public static final class ElevatorConstants {
    // CAN ID's
    private static final int ELEVATOR_LEFT_CAN_ID = 20;
    private static final int ELEVATOR_RIGHT_CAN_ID = 21;

    // Gear ratio
    private static final double ELEVATOR_GEAR_RATIO = 12.0;
    private static final double ELEVATOR_SPROCKET_DIAMETER = 1.504;
    private static final double ELEVATOR_STAGE_RATIO = 2.0;
    private static final double ELEVATOR_INCHES_PER_ROTATION = (ELEVATOR_SPROCKET_DIAMETER * Math.PI * ELEVATOR_STAGE_RATIO) / ELEVATOR_GEAR_RATIO;

    // Maximum and minimum extension of the elevator, in inches
    private static final double MAX_HEIGHT_INCHES = 88;
    public static final double MIN_HEIGHT_INCHES = 38.5;
    public static final double MIN_HEIGHT_MATCH = MIN_HEIGHT_INCHES + 0.25;
    private static final double MAX_ERROR_INCHES = 0.50;
    // Maximum and minimum extension of the elevator, in rotations
    private static final double MAX_HEIGHT_ROTATIONS = MAX_HEIGHT_INCHES / ELEVATOR_INCHES_PER_ROTATION;
    private static final double MIN_HEIGHT_ROTATIONS = MIN_HEIGHT_INCHES / ELEVATOR_INCHES_PER_ROTATION;

    private static final double MIN_MATCH_HEIGHT_ROTATIONS = (MIN_HEIGHT_MATCH) / ELEVATOR_INCHES_PER_ROTATION; 

    // Current limit of either motor
    private static final Current CURRENT_LIMIT = Amps.of(80);
    private static final CurrentLimitsConfigs ELEVATOR_CURRENT_CONFIG = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(CURRENT_LIMIT)
        .withSupplyCurrentLimit(CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true);

    private static final SoftwareLimitSwitchConfigs ELEVATOR_SOFT_LIMITS_CONFIG = new SoftwareLimitSwitchConfigs()
        .withReverseSoftLimitThreshold(ElevatorConstants.MIN_HEIGHT_ROTATIONS)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ElevatorConstants.MAX_HEIGHT_ROTATIONS)
        .withForwardSoftLimitEnable(true);

    // Static, Voltage, Gravity, and PID for the motor
    private static final double LEFT_KG = 0.30;
    private static final double LEFT_KS = 0;
    private static final double LEFT_KV = 0.0;
    private static final double LEFT_KA = 0;
    private static final double LEFT_KP = 16.0;
    private static final double LEFT_KI = 0;
    private static final double LEFT_KD = 0;
    private static final Slot0Configs LEFT_SLOT_CONFIG = new Slot0Configs()
        .withKG(LEFT_KG)
        .withKS(LEFT_KS)
        .withKV(LEFT_KV)
        .withKA(LEFT_KA)
        .withKP(LEFT_KP)
        .withKI(LEFT_KI)
        .withKD(LEFT_KD);

    private static final MotorOutputConfigs LEFT_MOTOR_CONFIGS = new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);

    private static final double RIGHT_KG = LEFT_KG;
    private static final double RIGHT_KS = LEFT_KS;
    private static final double RIGHT_KV = LEFT_KV;
    private static final double RIGHT_KA = LEFT_KA;
    private static final double RIGHT_KP = LEFT_KP;
    private static final double RIGHT_KI = LEFT_KI;
    private static final double RIGHT_KD = LEFT_KD;
    private static final Slot0Configs RIGHT_SLOT_CONFIG = new Slot0Configs()
        .withKG(RIGHT_KG)
        .withKS(RIGHT_KS)
        .withKV(RIGHT_KV)
        .withKA(RIGHT_KA)
        .withKP(RIGHT_KP)
        .withKI(RIGHT_KI)
        .withKD(RIGHT_KD);

    private static final MotorOutputConfigs RIGHT_MOTOR_CONFIGS = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);

    // Positions of the different Coral Branches in relation to the robot

    public static final Distance STATION_POSITION = Inches.of(38.75);
    public static final Distance L1_POSITION = Inches.of(38.75);
    public static final Distance L2_POSITION = Inches.of(38.75);
    public static final Distance L3_POSITION = Inches.of(54.0);
    public static final Distance L4_POSITION = Inches.of(78.0); // 81.5 was sometimes working

    // For trapezoid profile constrants.
    /** Maximum velocity of the Motors, in Rotations per second */
    private static final double MAX_VELOCITY_RPS = 500;

    /** Maximum acceleration of the Motors, in Rotations per second per second */
    private static final double MAX_ACCEL = 500;
    private static final double MAX_JERK = 400;

    private static final MotionMagicConfigs ELEVATOR_MOTION_CONFIGS = new MotionMagicConfigs()
    .withMotionMagicCruiseVelocity(MAX_VELOCITY_RPS)
    .withMotionMagicAcceleration(MAX_ACCEL)
    .withMotionMagicJerk(MAX_JERK);

    private static final Voltage MAX_VOLTS = Volts.of(11);

    private static final VoltageConfigs ELEVATOR_VOLTAGE_CONFIGS = new VoltageConfigs()
    .withPeakForwardVoltage(MAX_VOLTS)
    .withPeakReverseVoltage(MAX_VOLTS);
  }


  /** Creates a new Elevator. */
  public ElevatorSubsystem() {
    // Sets the motors to their CAN ID's
    elevatorLeftMotor = new TalonFX(ElevatorConstants.ELEVATOR_LEFT_CAN_ID);
    elevatorRightMotor = new TalonFX(ElevatorConstants.ELEVATOR_RIGHT_CAN_ID);

    // Set controller to default.
    elevatorLeftMotor.getConfigurator().apply(new TalonFXConfiguration());
    elevatorRightMotor.getConfigurator().apply(new TalonFXConfiguration());

    elevatorFollower = new Follower(elevatorLeftMotor.getDeviceID(), true);
    elevatorRightMotor.setControl(elevatorFollower);

    // Sets the configuration of the motors
    elevatorLeftConfig = new TalonFXConfiguration()
        .withSlot0(ElevatorConstants.LEFT_SLOT_CONFIG)
        .withCurrentLimits(ElevatorConstants.ELEVATOR_CURRENT_CONFIG)
        .withSoftwareLimitSwitch(ElevatorConstants.ELEVATOR_SOFT_LIMITS_CONFIG)
        .withMotorOutput(ElevatorConstants.LEFT_MOTOR_CONFIGS)
        .withMotionMagic(ElevatorConstants.ELEVATOR_MOTION_CONFIGS)
        .withVoltage(ElevatorConstants.ELEVATOR_VOLTAGE_CONFIGS);

    elevatorRightConfig = new TalonFXConfiguration()
        .withSlot0(ElevatorConstants.RIGHT_SLOT_CONFIG)
        .withCurrentLimits(ElevatorConstants.ELEVATOR_CURRENT_CONFIG)
        .withSoftwareLimitSwitch(ElevatorConstants.ELEVATOR_SOFT_LIMITS_CONFIG)
        .withMotorOutput(ElevatorConstants.RIGHT_MOTOR_CONFIGS)
        .withMotionMagic(ElevatorConstants.ELEVATOR_MOTION_CONFIGS)
        .withVoltage(ElevatorConstants.ELEVATOR_VOLTAGE_CONFIGS);

    // Go to a position with an added voltage for Feed Forward Compensation
    elevatorRequest = new MotionMagicVoltage(0);
    voltageRequest = new VoltageOut(Volts.of(0));

    // Applying the Configurators to their respective motors
    RobotContainer.applyTalonConfigs(elevatorLeftMotor, elevatorLeftConfig);
    RobotContainer.applyTalonConfigs(elevatorRightMotor, elevatorRightConfig);

    elevatorLeftMotor.setPosition(
        Rotations.of(ElevatorConstants.MIN_HEIGHT_INCHES / ElevatorConstants.ELEVATOR_INCHES_PER_ROTATION));
    elevatorRightMotor.setPosition(
        Rotations.of(ElevatorConstants.MIN_HEIGHT_INCHES / ElevatorConstants.ELEVATOR_INCHES_PER_ROTATION));

    SmartDashboard.putData("Elevator", this);
    SmartDashboard.putNumber("kG", 0);
  }

  // Get methods \\

  /**
   * Gets the current rotation of desired motor in rotations
   * 
   * @param rotations - the motor you want rotations of
   */
  private double getElevatorRotations() {
    return elevatorLeftMotor.getPosition().refresh().getValue().magnitude();
  }
  /**
   * @param motor
   * @return height in inches
   */
  public double getElevatorHeight() {
    return (ElevatorConstants.ELEVATOR_INCHES_PER_ROTATION * getElevatorRotations());
  }

  private boolean withinPositionError(double position) {
    return (Math.abs(getElevatorHeight() - position) < ElevatorConstants.MAX_ERROR_INCHES);
  }

  /**
   * Stops the elevator from moving
   */
  public Command stopElevator() {
    return runOnce(() -> {
      elevatorLeftMotor.stopMotor();
      elevatorRightMotor.stopMotor();
  });
  }

  public Command setElevatorPositionCommand(DoubleSupplier position) { 
    return 
    run(() -> {
      elevatorLeftMotor
      .setControl(elevatorRequest.withPosition(position.getAsDouble() / ElevatorConstants.ELEVATOR_INCHES_PER_ROTATION));
        })
        .until(() -> withinPositionError(position.getAsDouble()));
  }

  public Command setElevatorPositionCommand(double position) { 
    return 
    run(() -> {
      elevatorLeftMotor
      .setControl(elevatorRequest.withPosition(position / ElevatorConstants.ELEVATOR_INCHES_PER_ROTATION));
        })
        .until(() -> withinPositionError(position));
  }

  public Command setElevatorVoltage(double voltage) {
    return run(() -> {
      elevatorLeftMotor.setVoltage(voltage);
      elevatorRightMotor.setVoltage(voltage);
    });
  }

  public Command resetElevator() {
    SoftwareLimitSwitchConfigs config = new SoftwareLimitSwitchConfigs();
    return runOnce(() -> {
      RobotContainer.applyTalonConfigs(elevatorLeftMotor, elevatorLeftConfig.withSoftwareLimitSwitch(config));
      RobotContainer.applyTalonConfigs(elevatorRightMotor, elevatorRightConfig.withSoftwareLimitSwitch(config));
    }).andThen(setElevatorVoltage(-.5))
    .finallyDo(() -> {
    // stopElevator();
    setElevatorVoltage(0);
    elevatorLeftMotor.setPosition(ElevatorConstants.MIN_HEIGHT_ROTATIONS);
    elevatorRightMotor.setPosition(ElevatorConstants.MIN_HEIGHT_ROTATIONS);
    RobotContainer.applyTalonConfigs(elevatorLeftMotor, elevatorLeftConfig.withSoftwareLimitSwitch(ElevatorConstants.ELEVATOR_SOFT_LIMITS_CONFIG));
    RobotContainer.applyTalonConfigs(elevatorRightMotor, elevatorRightConfig.withSoftwareLimitSwitch(ElevatorConstants.ELEVATOR_SOFT_LIMITS_CONFIG));
    });
  }

  public Command stupidStupid() {
    return runOnce(() -> {
    elevatorLeftMotor.setPosition(ElevatorConstants.MIN_MATCH_HEIGHT_ROTATIONS);
    elevatorRightMotor.setPosition(ElevatorConstants.MIN_MATCH_HEIGHT_ROTATIONS);
  });
  }

  @Override
  public void periodic() {
  }

  @Override
  public void initSendable(SendableBuilder builder) {

    builder.addDoubleProperty("L Elevator Current Position Inches", () ->
    getElevatorHeight(), null);

    builder.addDoubleProperty("R Elevator Current Position Inches", () ->
    getElevatorHeight(), null);

    builder.addDoubleProperty("L Elevator Stator Current", () -> elevatorLeftMotor.getStatorCurrent().getValueAsDouble(), null);
    builder.addDoubleProperty("R Elevator Stator Current", () -> elevatorRightMotor.getStatorCurrent().getValueAsDouble(), null);

    builder.addDoubleProperty("L Elevator Velocity", () -> elevatorLeftMotor.getVelocity().getValueAsDouble(), null);
    super.initSendable(builder);
  }
}
