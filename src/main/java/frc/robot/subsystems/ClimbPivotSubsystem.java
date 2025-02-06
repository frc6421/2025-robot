// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbPivotSubsystem extends SubsystemBase {

  private final TalonFX leftPivotMotor;
  private final TalonFX rightPivotMotor;

  private final TalonFXConfiguration leftPivotMotorConfig;
  private final TalonFXConfiguration rightPivotMotorConfig;

  private final VoltageOut voltageRequest;
  public static class ClimbPivotConstants {

    private static final int LEFT_PIVOT_MOTOR_ID = 50;
    private static final int RIGHT_PIVOT_MOTOR_ID = 51;

    private static final double CLIMB_GEAR_RATIO = 150;

    private static final Current PIVOT_CURRENT_LIMIT = Amps.of(80); // TODO: Update Numbers
    private static final CurrentLimitsConfigs PIVOT_CURRENT_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(PIVOT_CURRENT_LIMIT)
        .withSupplyCurrentLimit(PIVOT_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true);

    // soft limits
    public static final double RIGHT_PIVOT_FORWARD_SOFT_LIMIT = 75; // TODO: Update Numbers
    public static final double RIGHT_PIVOT_REVERSE_SOFT_LIMIT = 0; // TODO: Update Numbers

    private static final FeedbackConfigs CLIMB_FEEDBACK_CONFIGS = new FeedbackConfigs()
        .withSensorToMechanismRatio(ClimbPivotConstants.CLIMB_GEAR_RATIO);

    private static final SoftwareLimitSwitchConfigs RIGHT_PIVOT_SOFT_LIMIT_CONFIGS = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitThreshold(ClimbPivotConstants.RIGHT_PIVOT_FORWARD_SOFT_LIMIT)
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(ClimbPivotConstants.RIGHT_PIVOT_REVERSE_SOFT_LIMIT)
        .withReverseSoftLimitEnable(true);

    private static final MotorOutputConfigs LEFT_PIVOT_MOTOR_CONFIGS = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    private static final MotorOutputConfigs RIGHT_PIVOT_MOTOR_CONFIGS = new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

  }

  /** Creates a new ClimbSubsystem. */
  public ClimbPivotSubsystem() {
    leftPivotMotor = new TalonFX(ClimbPivotConstants.LEFT_PIVOT_MOTOR_ID);
    rightPivotMotor = new TalonFX(ClimbPivotConstants.RIGHT_PIVOT_MOTOR_ID);

    leftPivotMotorConfig = new TalonFXConfiguration()
        .withCurrentLimits(ClimbPivotConstants.PIVOT_CURRENT_CONFIGS)
        .withMotorOutput(ClimbPivotConstants.LEFT_PIVOT_MOTOR_CONFIGS)
        .withFeedback(ClimbPivotConstants.CLIMB_FEEDBACK_CONFIGS);

    rightPivotMotorConfig = new TalonFXConfiguration()
        .withCurrentLimits(ClimbPivotConstants.PIVOT_CURRENT_CONFIGS)
        .withMotorOutput(ClimbPivotConstants.RIGHT_PIVOT_MOTOR_CONFIGS)
        .withSoftwareLimitSwitch(ClimbPivotConstants.RIGHT_PIVOT_SOFT_LIMIT_CONFIGS)
        .withFeedback(ClimbPivotConstants.CLIMB_FEEDBACK_CONFIGS);

    rightPivotMotor.getConfigurator().apply(leftPivotMotorConfig);
    leftPivotMotor.getConfigurator().apply(rightPivotMotorConfig);

    Follower follower = new Follower(rightPivotMotor.getDeviceID(), true);
    leftPivotMotor.setControl(follower);

    voltageRequest = new VoltageOut(0);

    SmartDashboard.putData("Climb Pivots", this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command setVoltageCommand(double voltage) {
    return this.run(() -> rightPivotMotor.setControl(voltageRequest.withOutput(voltage)));
  }

  public boolean isInPosition(){
    return rightPivotMotor.getPosition().getValueAsDouble() >= ClimbPivotConstants.RIGHT_PIVOT_FORWARD_SOFT_LIMIT;
  }

  public boolean isOutPosition(){
    return rightPivotMotor.getPosition().getValueAsDouble() >= ClimbPivotConstants.RIGHT_PIVOT_REVERSE_SOFT_LIMIT;
  }

  public void stopPivotMotors() {
    leftPivotMotor.stopMotor();
    rightPivotMotor.stopMotor();
  }

  public void intiSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Left Pivot Speed", () -> leftPivotMotor.get(), null);
    builder.addDoubleProperty("Right Pivot Speed", () -> rightPivotMotor.get(), null);
  }
}