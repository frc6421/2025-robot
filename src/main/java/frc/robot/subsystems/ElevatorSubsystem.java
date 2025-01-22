// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  // Creating the Motor Controller objects
  private TalonFX elevatorLeftMotor;
  private TalonFX elevatorRightMotor;

  // Creating the Motor Controller Configuration objects
  private final TalonFXConfiguration elevatorLeftConfig;
  private final TalonFXConfiguration elevatorRightConfig;
  private final Follower elevatorFollower;

  // Trapezoid Profile for the Elevator
  private TrapezoidProfile elevatorProfile;// Profile object
  private TrapezoidProfile.State elevatorSetPoint;// Set point
  private PositionVoltage elevatorRequest;// Voltage to maintian the set point

  private final Mechanism2d elevator2d;// Mechanism 2D object of the elevator
  private final MechanismRoot2d elevatorRoot;// Root for the Mechanism
  private final MechanismLigament2d elevatorLigament;// Ligament for the elevator
  private double elevatorHeight;// Length of the Elevator

  private final SendableChooser<TrapezoidProfile.State> elevatorChooser;

  private static final class ElevatorConstants {
    // CAN ID's
    private static final int ELEVATOR_LEFT_CAN_ID = 20;
    private static final int ELEVATOR_RIGHT_CAN_ID = 21;

    // Gear ratio
    private static final double ELEVATOR_GEAR_RATIO = 12.0;
    private static final Distance ELEVATOR_SPROCKET_DIAMETER = Inches.of(1.5); // TODO confirm this

    // How many rotations are needed to make the elevator move 1 meter in any
    // direction
    private static final double ROTATIONS_PER_METER = 1
        / (ELEVATOR_GEAR_RATIO * ELEVATOR_SPROCKET_DIAMETER.magnitude());

    // Maximum and minimum extension of the elevator, in meters
    private static final Distance MAX_HEIGHT = Inches.of(0); // TODO get value
    private static final Distance MIN_HEIGHT = Inches.of(0); // TODO get value

    // Current limit of either motor
    private static final Current CURRENT_LIMIT = Amps.of(0);
    private static final CurrentLimitsConfigs ELEVATOR_CURRENT_CONFIG = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(CURRENT_LIMIT)
        .withSupplyCurrentLimit(CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true);

    // Static, Voltage, Gravity, and PID for the motor
    private static final double LEFT_KG = 0;
    private static final double LEFT_KS = 0;
    private static final double LEFT_KV = 0;
    private static final double LEFT_KP = 0;
    private static final double LEFT_KI = 0;
    private static final double LEFT_KD = 0;
    private static final Slot0Configs LEFT_SLOT_CONFIG = new Slot0Configs()
        .withKG(LEFT_KG)
        .withKS(LEFT_KS)
        .withKV(LEFT_KV)
        .withKI(LEFT_KP)
        .withKD(LEFT_KI)
        .withKD(LEFT_KD);

    private static final MotorOutputConfigs LEFT_MOTOR_CONFIGS = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    private static final double RIGHT_KG = 0;
    private static final double RIGHT_KS = 0;
    private static final double RIGHT_KV = 0;
    private static final double RIGHT_KP = 0;
    private static final double RIGHT_KI = 0;
    private static final double RIGHT_KD = 0;
    private static final Slot0Configs RIGHT_SLOT_CONFIG = new Slot0Configs()
        .withKG(RIGHT_KG)
        .withKS(RIGHT_KS)
        .withKV(RIGHT_KV)
        .withKI(RIGHT_KP)
        .withKD(RIGHT_KI)
        .withKD(RIGHT_KD);

    private static final MotorOutputConfigs RIGHT_MOTOR_CONFIGS = new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    // Positions of the different Coral Branches in relation to the robot

    private static final Distance STATION_POSITION = Inches.of(0);
    private static final Distance L1_POSITION = Inches.of(0);
    private static final Distance L2_POSITION = Inches.of(0);
    private static final Distance L3_POSITION = Inches.of(0);
    private static final Distance L4_POSITION = Inches.of(0);

    // Maximum velocity of the Motors, in Rotations per Second
    private static final double MAX_VELOCITY_RPS = 0;

    // Maximum acceleration of the Motors, in Rotations per Second
    private static final double MAX_ACCEL_RPS = 0;

    // Creates new set states for the Trapezoid Profile
    private static final TrapezoidProfile.State BOTTOM_GOAL = new TrapezoidProfile.State(MIN_HEIGHT.magnitude(), 0);
    private static final TrapezoidProfile.State STATION_GOAL = new TrapezoidProfile.State(STATION_POSITION.magnitude(), 0);
    private static final TrapezoidProfile.State L1_GOAL = new TrapezoidProfile.State(L1_POSITION.magnitude(), 0);
    private static final TrapezoidProfile.State L2_GOAL = new TrapezoidProfile.State(L2_POSITION.magnitude(), 0);
    private static final TrapezoidProfile.State L3_GOAL = new TrapezoidProfile.State(L3_POSITION.magnitude(), 0);
    private static final TrapezoidProfile.State L4_GOAL = new TrapezoidProfile.State(L4_POSITION.magnitude(), 0);
    //TODO do we need algae states?
  }

  /** Creates a new Elevator. */
  public ElevatorSubsystem() {
    // Sets the motors to their CAN ID's
    elevatorLeftMotor = new TalonFX(ElevatorConstants.ELEVATOR_LEFT_CAN_ID);
    elevatorRightMotor = new TalonFX(ElevatorConstants.ELEVATOR_RIGHT_CAN_ID);

    elevatorFollower = new Follower(elevatorLeftMotor.getDeviceID(), true);
    elevatorRightMotor.setControl(elevatorFollower); // TODO does a follower need to apply confis, verify in CTRE example code

    // Sets the configuration of the motors
    elevatorLeftConfig = new TalonFXConfiguration()
        .withSlot0(ElevatorConstants.LEFT_SLOT_CONFIG)
        .withCurrentLimits(ElevatorConstants.ELEVATOR_CURRENT_CONFIG)
        //.withSoftwareLimitSwitch(null) // TODO MAKE LIMITS
        .withMotorOutput(ElevatorConstants.LEFT_MOTOR_CONFIGS);

    elevatorRightConfig = new TalonFXConfiguration()
        .withSlot0(ElevatorConstants.RIGHT_SLOT_CONFIG)
        .withCurrentLimits(ElevatorConstants.ELEVATOR_CURRENT_CONFIG)
        //.withSoftwareLimitSwitch(null) // TODO MAKE LIMITS
        .withMotorOutput(ElevatorConstants.RIGHT_MOTOR_CONFIGS);

    // Go to a position with an added voltage for Feed Forward Compensation
    elevatorRequest = new PositionVoltage(0).withSlot(0);

    // New Elevator Set point
    elevatorSetPoint = new TrapezoidProfile.State();

    // Setting the current Elevator length to the minimum extension
    elevatorHeight = ElevatorConstants.MIN_HEIGHT.magnitude();

    // Applying the Configurators to their respective motors
    elevatorLeftMotor.getConfigurator().apply(elevatorLeftConfig);
    elevatorRightMotor.getConfigurator().apply(elevatorRightConfig);

    // Creating and applying the Elevator Trapezoid profile
    elevatorProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY_RPS, ElevatorConstants.MAX_ACCEL_RPS));

   
    // Mechanism 2d
    elevator2d = new Mechanism2d(1, 1);// Creates a canvase of width 1 and height 1
    elevatorRoot = elevator2d.getRoot("Base", 0.5, 0);// Name of the root with a position of (0.5,0)

    // Appends a mechanism to the Elevator Ligament and creates a new Ligament 2D
    elevatorLigament = elevatorRoot.append(new MechanismLigament2d("ElevatorExt", elevatorHeight, 90));

    elevatorChooser = new SendableChooser<TrapezoidProfile.State>();
    elevatorChooser.addOption("-----L4-----", ElevatorConstants.L4_GOAL);
    elevatorChooser.addOption("-----L3-----", ElevatorConstants.L3_GOAL);
    elevatorChooser.addOption("-----L2-----", ElevatorConstants.L2_GOAL);
    elevatorChooser.addOption("-----L1-----", ElevatorConstants.L1_GOAL);
    elevatorChooser.setDefaultOption("Bottom", ElevatorConstants.BOTTOM_GOAL);
    elevatorChooser.addOption("Coral Station", ElevatorConstants.STATION_GOAL);
    

    SmartDashboard.putData("Elevator", this);
    SmartDashboard.putData("Elevator/Elevator2d", elevator2d);
    SmartDashboard.putData("Elevator/ElevatorChooser", elevatorChooser);
    SmartDashboard.putString("Elevator/ElevatorSetPoint", elevatorSetPoint.toString());
  }

  /**
   * Gets the current rotation of desired motor in rotations
   * 
   * @param rotations - the motor you want rotations of
   */
  /**
   * @breif Gets the average rotation of the Elevator Motors
   * @return The average of the Elevator Motors current rotations
   */
  private Angle getElevatorRotations() {
    return elevatorLeftMotor.getPosition().getValue();
  }

  /**
   * @breif Stops the elevator from moving
   */
  public void stopElevator() {
    elevatorLeftMotor.stopMotor();
  }


  /**
   * @breif Sets the position, rotation, and velocity of the Elevator's Trapezoid
   *        Profile
   * @param goal The goal for the Trapezoid Profile. Given from the
   *             TrapezoidProfile.State, where the possible options are
   *             STATION_GOAL, BOTTOM
   *             L1_GOAL, L2_GOAL, L3_GOAL, L4_Goal
   */
  public void applyElevatorProfile(TrapezoidProfile.State goal) {
    elevatorSetPoint = elevatorProfile.calculate(.020, elevatorSetPoint, goal);
    elevatorRequest.Position = elevatorSetPoint.position;
    elevatorRequest.Velocity = elevatorSetPoint.velocity;

    /// Applies the Position and Velocity to the motors
    elevatorLeftMotor.setControl(elevatorRequest);
  }

  /**
   * Set the elevator position in rotations
   * 
   * @param rotations - number of {@code}rotations{@code} you want the elevator to
   *                  go
   */
  /**
   * @breif Sets the elevator position, in rotations
   * @param rotations Number of rotations to move the Elevator by
   */
  public void setElevatorPosition(double rotations) {
    elevatorLeftMotor.setControl(elevatorRequest.withPosition(rotations));
  }

  private Distance getElevatorHeight() {
    return Meters.of(getElevatorRotations().magnitude() * ElevatorConstants.ROTATIONS_PER_METER);
  }

  public TrapezoidProfile.State getElevatorChooser() {
    return elevatorChooser.getSelected();
  }

  public Command setElevatorPosition(TrapezoidProfile.State state) {

    final Timer timer = new Timer();
    
    return this.run(timer::restart, );
  }
  @Override
  public void periodic() {}

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Elevator Position Rot", () -> getElevatorRotations().magnitude(), null);
    builder.addDoubleProperty("Elevator Position Meters", () -> getElevatorHeight().magnitude(), null);
  }
}
