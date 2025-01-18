// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  //Creating the Motor Controller objects
  private TalonFX elevatorLeft;
  private TalonFX elevatorRight;
  //Creating the Motor Controller Configuration objects
  private final TalonFXConfiguration elevatorLeftConfig;
  private final TalonFXConfiguration elevatorRightConfig;
  //Trapezoid Profile for the Elevator
  private TrapezoidProfile elevatorProfile;//Profile object
  private TrapezoidProfile.State elevatorSetPoint;//Set point
  private PositionVoltage elevatorRequest;//Voltage to maintian the set point

  private final Mechanism2d elevator2d;//Mechanism 2D object of the elevator
  private final MechanismRoot2d elevatorRoot;//Root for the Mechanism
  private final MechanismLigament2d elevatorLigament;//Ligament for the elevator
  private final ShuffleboardTab elevatorTab;//Shuffleboard tab for tuning
  private double elevatorLength;//Length of the Elevator
  
    public static final class ElevatorConstants {
      //CAN ID's
      public static final int ELEVATOR_LEFT_CAN_ID = 20;
      public static final int ELEVATOR_RIGHT_CAN_ID = 21;
      //Current limit of either motor
      public static final double CURRENT_LIMIT = 0;
      //Gear ratio 
      public static final double ELEVATOR_GEAR_RATIO = 12.0;
      //Elevator angle, for if something doesn't seem right
      public static final double ELEVATOR_ANGLE = 0;
      //How many rotations are needed to make the elevator move 1 meter in any direction
      public static final double ROTATIONS_PER_METER = 0;
      //Maximum and minimum extension of the elevator, in meters
      public static final double MAX_EXT = 0;
      public static final double MIN_EXT = 0;
      //Static, Voltage, Gravity, and PID for the motor
      public static final double LEFT_KS = 0;
      public static final double LEFT_KV = 0;
      public static final double LEFT_KP = 0;
      public static final double LEFT_KI = 0;
      public static final double LEFT_KD = 0;
      public static final double LEFT_KG = 0;
      public static final double RIGHT_KS = 0;
      public static final double RIGHT_KV = 0;
      public static final double RIGHT_KP = 0;
      public static final double RIGHT_KI = 0;
      public static final double RIGHT_KD = 0;
      public static final double RIGHT_KG = 0;
      //Positions of the different Coral Branches in relation to the robot
      public static final double STATION_POSITION = 0;
      public static final double L1_POSITION = 0;
      public static final double L2_POSITION = 0;
      public static final double L3_POSITION = 0;
      public static final double L4_POSITION = 0;
      //Maximum velocity of the Motors, in Rotations per Second
      public static final double MAX_VELOCITY_RPS = 0;
      //Maximum acceleration of the Motors, in Rotations per Second
      public static final double MAX_ACCEL_RPS = 0;
      //Creates new set states for the Trapezoid Profile to "acchieve"
      public static final TrapezoidProfile.State STATION_GOAL = new TrapezoidProfile.State(STATION_POSITION, 0);
      public static final TrapezoidProfile.State L1_GOAL = new TrapezoidProfile.State(L1_POSITION, 0);
      public static final TrapezoidProfile.State L2_GOAL = new TrapezoidProfile.State(L2_POSITION, 0);
      public static final TrapezoidProfile.State L3_GOAL = new TrapezoidProfile.State(L3_POSITION, 0);
      public static final TrapezoidProfile.State L4_GOAL = new TrapezoidProfile.State(L4_POSITION, 0);
    }
    /** Creates a new Elevator. */
    public Elevator() {
      //Sets the motors to their CAN ID's
      elevatorLeft = new TalonFX(ElevatorConstants.ELEVATOR_LEFT_CAN_ID);
      elevatorRight = new TalonFX(ElevatorConstants.ELEVATOR_RIGHT_CAN_ID);
      //Sets the configuration of the motors
      elevatorLeftConfig = new TalonFXConfiguration();
      elevatorRightConfig = new TalonFXConfiguration();
      //Go to a position with an added voltage for Feed Forward Compensation
      elevatorRequest = new PositionVoltage(0).withSlot(0);
      //New Elevator Set point
      elevatorSetPoint = new TrapezoidProfile.State();
      //Setting the CTRE Inverted and Neutral modes
      elevatorLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      elevatorRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      elevatorLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      elevatorRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      //Setting the current Elevator length to the minimum extension
      elevatorLength = ElevatorConstants.MIN_EXT;
      
      //Setting the Static, Voltage, Gravity, and PID values to the motor Configurations
      elevatorLeftConfig.Slot0.kS = ElevatorConstants.LEFT_KS;
      elevatorLeftConfig.Slot0.kV = ElevatorConstants.LEFT_KV;
      elevatorLeftConfig.Slot0.kP = ElevatorConstants.LEFT_KP;
      elevatorLeftConfig.Slot0.kI = ElevatorConstants.LEFT_KI;
      elevatorLeftConfig.Slot0.kD = ElevatorConstants.LEFT_KD;
      elevatorLeftConfig.Slot0.kG = ElevatorConstants.LEFT_KG;
      
      elevatorRightConfig.Slot0.kS = ElevatorConstants.RIGHT_KS;
      elevatorRightConfig.Slot0.kV = ElevatorConstants.RIGHT_KV;
      elevatorRightConfig.Slot0.kP = ElevatorConstants.RIGHT_KP;
      elevatorRightConfig.Slot0.kI = ElevatorConstants.RIGHT_KI;
      elevatorRightConfig.Slot0.kD = ElevatorConstants.RIGHT_KD;
      elevatorRightConfig.Slot0.kG = ElevatorConstants.RIGHT_KG;
      
      //Setting and applying the Stator current limits to the motors
      elevatorLeftConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.CURRENT_LIMIT;
      elevatorLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      elevatorRightConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.CURRENT_LIMIT;
      elevatorRightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  
      //Applying the Configurators to their respective motors
      elevatorLeft.getConfigurator().apply(elevatorLeftConfig);
      elevatorRight.getConfigurator().apply(elevatorRightConfig);
  
      //Creating and applying the Elevator Trapezoid profile
      elevatorProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY_RPS, ElevatorConstants.MAX_ACCEL_RPS)
      );
      //Shuffleboard stuff
      elevatorTab = Shuffleboard.getTab("Elevator");//Creating Tab
      elevatorTab.add(this);//I don't know what this truly does, but I think it applies all the stuff set to Shuffleboard
      //Mechanism 2d
      elevator2d = new Mechanism2d(1, 1);//Creates a canvase of width 1 and height 1
      elevatorRoot = elevator2d.getRoot("Base", 0.5, 0);//Name of the root with a position of (0.5,0)
      //Appends a mechanism to the Elevator Ligament and creates a new Ligament 2D
      elevatorLigament = elevatorRoot.append(new MechanismLigament2d("ElevatorExt", elevatorLength, ElevatorConstants.ELEVATOR_ANGLE));
      elevatorTab.add("Elevator2d", elevator2d);//Adds the Elevator 2D to the Shuffleboard
    }
  
    /**Gets the current rotation of desired motor in rotations
     * @param rotations - the motor you want rotations of
     */
    /**
     * @breif   Gets the average rotation of the Elevator Motors
     * @return  The average of the Elevator Motors current rotations
     */
    public double getElevatorRotations() {
      return ((elevatorLeft.getPosition().getValueAsDouble() + elevatorRight.getPosition().getValueAsDouble()) / 2);
    }
  
    /**
     * @breif   Stops the elevator from moving
     */
    public void stopElevator() {
      elevatorLeft.stopMotor();
      elevatorRight.stopMotor();
    }
  
    /**
     * @breif   Sets the Elevator Trapezoid Profile Set Point
     * @param goal  The goal for the Trapezoid Profile. Given from the TrapezoidProfile.State, where the possible options are STATION_GOAL,
     * L1_GOAL, L2_GOAL, L3_GOAL, L4_Goal
     */
    public void setElevatorSetPoint(TrapezoidProfile.State goal) {
     elevatorSetPoint = elevatorProfile.calculate(.020, elevatorSetPoint, goal);
    }
  
    /**
     * @breif   Sets the position, rotation, and velocity of the Elevator's Trapezoid Profile 
     * @param goal  The goal for the Trapezoid Profile. Given from the TrapezoidProfile.State, where the possible options are STATION_GOAL,
     * L1_GOAL, L2_GOAL, L3_GOAL, L4_Goal
     */
    public void applyElevatorProfile(TrapezoidProfile.State goal) {
      setElevatorSetPoint(goal);
      elevatorRequest.Position = elevatorSetPoint.position;
      elevatorRequest.Velocity = elevatorSetPoint.velocity;
  
      ///Applies the Position and Velocity to the motors
      elevatorLeft.setControl(elevatorRequest);
      elevatorRight.setControl(elevatorRequest);
    }
  
    /**Set the elevator position in rotations
     * @param rotations - number of {@code}rotations{@code} you want the elevator to go
     */
    /**
     * @breif   Sets the elevator position, in rotations
     * @param rotations   Number of rotations to move the Elevator by
     */
    public void setElevatorPosition(double rotations) {
      elevatorLeft.setControl(elevatorRequest.withPosition(rotations));
      elevatorRight.setControl(elevatorRequest.withPosition(rotations));
    }
  
    @Override
    public void periodic() {
      //Constantly updates the current lenght of the Elevator
      elevatorLength = getElevatorRotations() * ElevatorConstants.ROTATIONS_PER_METER;
    }

  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addDoubleProperty("Elevator Position Rot", this::getElevatorRotations, null);
      builder.addDoubleProperty("Elevator Position Meters", () -> elevatorLength,  null);
  }
}
