// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class ElevatorSubsystem extends SubsystemBase {
  // Creating the Motor Controller objects
  private final TalonFX elevatorLeftMotor;
  private final TalonFX elevatorRightMotor;

  // Creating the Motor Controller Configuration objects
  private final TalonFXConfiguration elevatorLeftConfig;
  private final TalonFXConfiguration elevatorRightConfig;
  private final Follower elevatorFollower;

  // Trapezoid Profile for the Elevator
  private TrapezoidProfile elevatorProfile;// Profile object
  private TrapezoidProfile.State currentElevatorState;// Set point
  private MotionMagicVoltage elevatorRequest;// Voltage to maintian the set point
  private VoltageOut voltageRequest;

  private final SendableChooser<TrapezoidProfile.State> elevatorChooser;

  private final SysIdRoutine sysIdRoutine;

  public static final class ElevatorConstants {
    // CAN ID's
    private static final int ELEVATOR_LEFT_CAN_ID = 20;
    private static final int ELEVATOR_RIGHT_CAN_ID = 21;

    // Gear ratio
    private static final double ELEVATOR_GEAR_RATIO = 12.0;
    private static final double ELEVATOR_SPROCKET_DIAMETER = 1.504;
    private static final double ELEVATOR_STAGE_RATIO = 3.0;
    private static final double ELEVATOR_INCHES_PER_ROTATION = (ELEVATOR_SPROCKET_DIAMETER * Math.PI * ELEVATOR_STAGE_RATIO) / ELEVATOR_GEAR_RATIO;

    private static final double METERS_PER_ROTATION = Units.inchesToMeters(ELEVATOR_INCHES_PER_ROTATION);
    private static final double MAX_ERROR_MOTION = 1.02;

    // Maximum and minimum extension of the elevator, in meters
    private static final double MAX_HEIGHT = Units.inchesToMeters(65.271);
    public static final double MIN_HEIGHT = Units.inchesToMeters(29.271);
    // Maximum and minimum extension of the elevator, in rotations
    private static final double MAX_HEIGHT_ROTATIONS = MAX_HEIGHT / METERS_PER_ROTATION;
    private static final double MIN_HEIGHT_ROTATIONS = MIN_HEIGHT / METERS_PER_ROTATION;

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
    private static final double LEFT_KG = 0.29;
    private static final double LEFT_KS = 0;
    private static final double LEFT_KV = 0.1 * ElevatorConstants.ELEVATOR_INCHES_PER_ROTATION;
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
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

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
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    // Positions of the different Coral Branches in relation to the robot

    public static final Distance STATION_POSITION = Inches.of(32);
    public static final Distance L1_POSITION = Inches.of(34);
    public static final Distance L2_POSITION = Inches.of(36);
    public static final Distance L3_POSITION = Inches.of(38);
    public static final Distance L4_POSITION = Inches.of(40);

    // For trapezoid profile constrants.
    /** Maximum velocity of the Motors, in Rotations per second */
    private static final double MAX_VELOCITY_RPS = 90;

    /** Maximum acceleration of the Motors, in Rotations per second per second */
    private static final double MAX_ACCEL = 85;
    private static final double MAX_JERK = 200;

    // Creates new set states for the Trapezoid Profile
    public static final TrapezoidProfile.State BOTTOM_GOAL = new TrapezoidProfile.State(MIN_HEIGHT_ROTATIONS, 0);
    private static final TrapezoidProfile.State STATION_GOAL = new TrapezoidProfile.State(STATION_POSITION.magnitude() / METERS_PER_ROTATION,
    0);
    public static final TrapezoidProfile.State L1_GOAL = new TrapezoidProfile.State(L1_POSITION.magnitude(), 0);
    private static final TrapezoidProfile.State L2_GOAL = new TrapezoidProfile.State(L2_POSITION.magnitude(), 0);
    private static final TrapezoidProfile.State L3_GOAL = new TrapezoidProfile.State(L3_POSITION.magnitude(), 0);
    private static final TrapezoidProfile.State L4_GOAL = new TrapezoidProfile.State(L4_POSITION.magnitude(), 0);
    // TODO do we need algae states?
    
    private static final Voltage MAX_VOLTS = Volts.of(11);
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
        .withMotorOutput(ElevatorConstants.LEFT_MOTOR_CONFIGS);
    
    elevatorLeftConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MAX_VELOCITY_RPS;
    elevatorLeftConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MAX_ACCEL;
    elevatorLeftConfig.MotionMagic.MotionMagicJerk = ElevatorConstants.MAX_JERK;

    elevatorLeftConfig.Voltage.withPeakForwardVoltage(ElevatorConstants.MAX_VOLTS.magnitude())
        .withPeakReverseVoltage(ElevatorConstants.MAX_VOLTS.magnitude());

    elevatorRightConfig = new TalonFXConfiguration()
        .withSlot0(ElevatorConstants.RIGHT_SLOT_CONFIG)
        .withCurrentLimits(ElevatorConstants.ELEVATOR_CURRENT_CONFIG)
        .withSoftwareLimitSwitch(ElevatorConstants.ELEVATOR_SOFT_LIMITS_CONFIG)
        .withMotorOutput(ElevatorConstants.RIGHT_MOTOR_CONFIGS);

    elevatorRightConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MAX_VELOCITY_RPS;
    elevatorRightConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MAX_ACCEL;
    elevatorRightConfig.MotionMagic.MotionMagicJerk = ElevatorConstants.MAX_JERK;

    elevatorRightConfig.Voltage.withPeakForwardVoltage(ElevatorConstants.MAX_VOLTS.magnitude())
        .withPeakReverseVoltage(ElevatorConstants.MAX_VOLTS.magnitude());

    // Go to a position with an added voltage for Feed Forward Compensation
    elevatorRequest = new MotionMagicVoltage(0);
    voltageRequest = new VoltageOut(Volts.of(0));

    // New Elevator Set point
    currentElevatorState = new TrapezoidProfile.State();

    // Applying the Configurators to their respective motors
    elevatorLeftMotor.getConfigurator().apply(elevatorLeftConfig);
    elevatorRightMotor.getConfigurator().apply(elevatorRightConfig);

    elevatorLeftMotor.setPosition(
        Rotations.of(ElevatorConstants.MIN_HEIGHT / ElevatorConstants.METERS_PER_ROTATION));
    elevatorRightMotor.setPosition(
        Rotations.of(ElevatorConstants.MIN_HEIGHT / ElevatorConstants.METERS_PER_ROTATION));

    // Creating and applying the Elevator Trapezoid profile
    elevatorProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY_RPS, 0));

    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(1).per(Second),
            Volts.of(4),
            Seconds.of(5),
            // Log state with SignalLogger class
            (state) -> SignalLogger.writeString("SysIdTranslation_StateElevator", state.toString())),
        new SysIdRoutine.Mechanism(
            output -> elevatorLeftMotor.setControl(voltageRequest.withOutput(output)),
            null,
            this));

    elevatorChooser = new SendableChooser<TrapezoidProfile.State>();
    elevatorChooser.addOption("-----L4-----", ElevatorConstants.L4_GOAL);
    elevatorChooser.addOption("-----L3-----", ElevatorConstants.L3_GOAL);
    elevatorChooser.addOption("-----L2-----", ElevatorConstants.L2_GOAL);
    elevatorChooser.addOption("-----L1-----", ElevatorConstants.L1_GOAL);
    elevatorChooser.setDefaultOption("Bottom", ElevatorConstants.BOTTOM_GOAL);
    elevatorChooser.addOption("Coral Station", ElevatorConstants.STATION_GOAL);

    SmartDashboard.putData("Elevator", this);
    SmartDashboard.putData("Elevator/ElevatorChooser", elevatorChooser);
  }

  // Get methods \\

  /**
   * Gets the current rotation of desired motor in rotations
   * 
   * @param rotations - the motor you want rotations of
   */
  private double getElevatorRotations(TalonFX motor) {
    return motor.getPosition().refresh().getValue().magnitude();
  }

  private double getElevatorHeight(TalonFX motor) {
    return (ElevatorConstants.METERS_PER_ROTATION * getElevatorRotations(motor));
  }

  private LinearVelocity getElevatorVelocity() {
    return MetersPerSecond.of(elevatorLeftMotor.getVelocity().refresh().getValueAsDouble()
        * ElevatorConstants.METERS_PER_ROTATION);
  }

  public TrapezoidProfile.State getSelectedState() {
    return elevatorChooser.getSelected();
  }

  // Set methods \\

  /**
   * Stops the elevator from moving
   */
  public void stopElevator() {
    elevatorLeftMotor.stopMotor();
  }

  /**
   * Set the elevator position in rotations
   * 
   * @param position number of inches to go
   */
  public void setElevatorPosition(double position) {
    //elevatorLeftMotor.setControl(elevatorRequest.withPosition(Units.inchesToMeters(position) / ElevatorConstants.METERS_PER_ROTATION).withVelocity(.1));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public void setMagicMotion(double position) {
    elevatorLeftMotor.setControl(elevatorRequest.withPosition(Units.inchesToMeters(position) / ElevatorConstants.METERS_PER_ROTATION));
  }

  public Command setElevatorPositionCommand(double position ) {

    return run(() -> {
      setMagicMotion(position);
        })
        .until(() -> getElevatorHeight(elevatorLeftMotor) < position * ElevatorConstants.MAX_ERROR_MOTION && 
        getElevatorHeight(elevatorLeftMotor) * ElevatorConstants.MAX_ERROR_MOTION > position);
  }

  public Command setElevatorVoltage(double voltage) {
    return run(() -> {
      elevatorLeftMotor.setVoltage(voltage);
    });
  }

  @Override
  public void periodic() {
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("L Elevator Current Position Rotations", () ->
    getElevatorRotations(elevatorLeftMotor), null);

    builder.addDoubleProperty("R Elevator Current Position Rotations", () ->
    getElevatorRotations(elevatorRightMotor), null);

    builder.addDoubleProperty("L Elevator Current Position Inches", () ->
    Units.metersToInches(getElevatorHeight(elevatorLeftMotor)), null);

    builder.addDoubleProperty("R Elevator Current Position Inches", () ->
    Units.metersToInches(getElevatorHeight(elevatorRightMotor)), null);

    builder.addDoubleProperty("Elevator Goal Position Meters", () ->
    getSelectedState().position, null);

    builder.addDoubleProperty("Elevator SetPoint", () -> currentElevatorState.position, null);
    builder.addDoubleProperty("Elevator Stator Current", () -> elevatorLeftMotor.getStatorCurrent().getValueAsDouble(), null);
  }
}
