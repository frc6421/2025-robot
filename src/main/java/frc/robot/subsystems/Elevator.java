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
  private TalonFX elevatorLeft;
  private TalonFX elevatorRight;

  private final TalonFXConfiguration elevatorLeftConfig;
  private final TalonFXConfiguration elevatorRightConfig;

  private TrapezoidProfile elevatorProfile;
  private TrapezoidProfile.State elevatorSetPoint;
  private PositionVoltage elevatorRequest;

  //Mechanism
  private final Mechanism2d elevator2d;
  private final MechanismRoot2d elevatorRoot;
  private final MechanismLigament2d elevatorLigament;
  private final ShuffleboardTab elevatorTab;
  private double elevatorLength;
  
    public static final class ElevatorConstants {
      public static final int ELEVATOR_LEFT_CAN_ID = 20;
      public static final int ELEVATOR_RIGHT_CAN_ID = 21;
      public static final double CURRENT_LIMIT = 0;
      public static final double ELEVATOR_GEAR_RATIO = 12.0;
      public static final double ELEVATOR_ANGLE = 0;
      public static final double ROTATIONS_PER_METER = 0;
      public static final double MAX_EXT = 0;
      public static final double MIN_EXT = 0;
  
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
  
      //Positions
      public static final double STATION_POSITION = 0;
      public static final double L1_POSITION = 0;
      public static final double L2_POSITION = 0;
      public static final double L3_POSITION = 0;
      public static final double L4_POSITION = 0;
  
      //Profile ROTATIONS PER SECOND
      public static final double MAX_VELOCITY_RPS = 0;
      public static final double MAX_ACCEL_RPS = 0;
  
      public static final TrapezoidProfile.State STATION_GOAL = new TrapezoidProfile.State(STATION_POSITION, 0);
      public static final TrapezoidProfile.State L1_GOAL = new TrapezoidProfile.State(L1_POSITION, 0);
      public static final TrapezoidProfile.State L2_GOAL = new TrapezoidProfile.State(L2_POSITION, 0);
      public static final TrapezoidProfile.State L3_GOAL = new TrapezoidProfile.State(L3_POSITION, 0);
      public static final TrapezoidProfile.State L4_GOAL = new TrapezoidProfile.State(L4_POSITION, 0);
    }
    /** Creates a new Elevator. */
    public Elevator() {
      elevatorLeft = new TalonFX(ElevatorConstants.ELEVATOR_LEFT_CAN_ID);
      elevatorRight = new TalonFX(ElevatorConstants.ELEVATOR_RIGHT_CAN_ID);
  
      elevatorLeftConfig = new TalonFXConfiguration();
      elevatorRightConfig = new TalonFXConfiguration();
  
      elevatorRequest = new PositionVoltage(0).withSlot(0);
      elevatorSetPoint = new TrapezoidProfile.State();
  
      elevatorLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      elevatorRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      elevatorLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      elevatorRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      elevatorLength = ElevatorConstants.MIN_EXT;
      
      //PID
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
      
      //Current Limits
      elevatorLeftConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.CURRENT_LIMIT;
      elevatorLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      elevatorRightConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.CURRENT_LIMIT;
      elevatorRightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  
      //Apply Configurators
      elevatorLeft.getConfigurator().apply(elevatorLeftConfig);
      elevatorRight.getConfigurator().apply(elevatorRightConfig);
  
      //Profile
      elevatorProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY_RPS, ElevatorConstants.MAX_ACCEL_RPS)
      );
      
      elevatorTab = Shuffleboard.getTab("Elevator");
      elevatorTab.add(this);
      //Mechanism 2d
      elevator2d = new Mechanism2d(1, 1);
      elevatorRoot = elevator2d.getRoot("Base", .5, 0);
      elevatorLigament = elevatorRoot.append(new MechanismLigament2d("ElevatorExt", elevatorLength, ElevatorConstants.ELEVATOR_ANGLE));
      elevatorTab.add("Elevator2d", elevator2d);
    }
  
    /**Gets the current rotation of desired motor in rotations
     * @param rotations - the motor you want rotations of
     */
    public double getElevatorRotations() {
      return ((elevatorLeft.getPosition().getValueAsDouble() + elevatorRight.getPosition().getValueAsDouble()) / 2);
    }
  
    /** Stops the elevator*/
    public void stopElevator() {
      elevatorLeft.stopMotor();
      elevatorRight.stopMotor();
    }
  
    /**Set the new Elevator TrapezoidProfile setpoint
     * @param goal - the end goal for the TrapezoidProfile
     */
    public void setElevatorSetPoint(TrapezoidProfile.State goal) {
     elevatorSetPoint = elevatorProfile.calculate(.020, elevatorSetPoint, goal);
    }
  
    /**Set the {@code}position (rot){@code} and 
     * {@code}velocity{@code} of the elevator's 
     * {@code}TrapezoidProfile{@code}, and then to the elevator */
    public void applyElevatorProfile(TrapezoidProfile.State goal) {
      setElevatorSetPoint(goal);
      elevatorRequest.Position = elevatorSetPoint.position;
      elevatorRequest.Velocity = elevatorSetPoint.velocity;
  
      //This line applies the velocity/position request to the actual motor
      elevatorLeft.setControl(elevatorRequest);
      elevatorRight.setControl(elevatorRequest);
    }
  
    /**Set the elevator position in rotations
     * @param rotations - number of {@code}rotations{@code} you want the elevator to go
     */
    public void setElevatorPosition(double rotations) {
      elevatorLeft.setControl(elevatorRequest.withPosition(rotations));
      elevatorRight.setControl(elevatorRequest.withPosition(rotations));
    }
  
    @Override
    public void periodic() {
      elevatorLength = getElevatorRotations() * ElevatorConstants.ROTATIONS_PER_METER;
      
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
      super.initSendable(builder);
      builder.addDoubleProperty("Elevator Position Rot", this::getElevatorRotations, null);
      builder.addDoubleProperty("Elevator Position Meters", () -> elevatorLength,  null);
  }
}
