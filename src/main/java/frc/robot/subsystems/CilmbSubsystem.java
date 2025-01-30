// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CilmbSubsystem extends SubsystemBase {

  private final TalonFX leftPivotMotor;
  private final TalonFX rightPivotMotor;

  private final TalonFXConfiguration leftPivotMotorConfig;
  private final TalonFXConfiguration rightPivotMotorConfig;

  public static class ClimbConstants {

    private static final int LEFT_PIVOT_MOTOR_ID = 50;
    private static final int RIGHT_PIVOT_MOTOR_ID = 51;

    private static final Current PIVOT_CURRENT_LIMIT = Amps.of(0); // TODO: Update Numbers
    private static final CurrentLimitsConfigs PIVOT_CURRENT_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(PIVOT_CURRENT_LIMIT)
        .withSupplyCurrentLimit(PIVOT_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true);


    // soft limits
    private static double LEFT_PIVOT_FORWARD_SOFT_LIMIT = 0; // TODO: Update Numbers
    private static double LEFT_PIVOT_REVERSE_SOFT_LIMIT = 0; // TODO: Update Numbers
    private static double RIGHT_PIVOT_FORWARD_SOFT_LIMIT = 0; // TODO: Update Numbers
    private static double RIGHT_PIVOT_REVERSE_SOFT_LIMIT = 0; // TODO: Update Numbers

    private static final SoftwareLimitSwitchConfigs LEFT_PIVOT_SOFT_LIMIT_CONFIGS = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitThreshold(ClimbConstants.LEFT_PIVOT_FORWARD_SOFT_LIMIT)
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(ClimbConstants.LEFT_PIVOT_REVERSE_SOFT_LIMIT)
        .withReverseSoftLimitEnable(true);

    private static final SoftwareLimitSwitchConfigs RIGHT_PIVOT_SOFT_LIMIT_CONFIGS = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitThreshold(ClimbConstants.RIGHT_PIVOT_FORWARD_SOFT_LIMIT)
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(ClimbConstants.RIGHT_PIVOT_REVERSE_SOFT_LIMIT)
        .withReverseSoftLimitEnable(true);

    private static final MotorOutputConfigs LEFT_PIVOT_MOTOR_CONFIGS = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    private static final MotorOutputConfigs RIGHT_PIVOT_MOTOR_CONFIGS = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    public static final Voltage LEFT_PIVOT_VOLTAGE = Volts.of(0); // TODO: Update Numbers
    public static final Voltage RIGHT_PIVOT_VOLTAGE = Volts.of(0); // TODO: Update Numbers

  }

  /** Creates a new CilmbSubsystem. */
  public CilmbSubsystem() {
    leftPivotMotor = new TalonFX(ClimbConstants.LEFT_PIVOT_MOTOR_ID);
    rightPivotMotor = new TalonFX(ClimbConstants.RIGHT_PIVOT_MOTOR_ID);

    leftPivotMotorConfig = new TalonFXConfiguration()
        .withCurrentLimits(ClimbConstants.PIVOT_CURRENT_CONFIGS)
        .withMotorOutput(ClimbConstants.LEFT_PIVOT_MOTOR_CONFIGS)
        .withSoftwareLimitSwitch(ClimbConstants.LEFT_PIVOT_SOFT_LIMIT_CONFIGS);

    rightPivotMotorConfig = new TalonFXConfiguration()
        .withCurrentLimits(ClimbConstants.PIVOT_CURRENT_CONFIGS)
        .withMotorOutput(ClimbConstants.RIGHT_PIVOT_MOTOR_CONFIGS)
        .withSoftwareLimitSwitch(ClimbConstants.RIGHT_PIVOT_SOFT_LIMIT_CONFIGS);

    rightPivotMotor.getConfigurator().apply(leftPivotMotorConfig);
    leftPivotMotor.getConfigurator().apply(rightPivotMotorConfig);

    SmartDashboard.putData("Climb Pivots", this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVoltage() {
    leftPivotMotor.setVoltage(ClimbConstants.LEFT_PIVOT_VOLTAGE.magnitude());
    rightPivotMotor.setVoltage(ClimbConstants.RIGHT_PIVOT_VOLTAGE.magnitude());
  }

  public void setPosition(double position) {
    leftPivotMotor.setPosition(ClimbConstants.LEFT_PIVOT_REVERSE_SOFT_LIMIT);
    rightPivotMotor.setPosition(ClimbConstants.RIGHT_PIVOT_REVERSE_SOFT_LIMIT);
  }


  public void stopPivotMotors() {
    leftPivotMotor.stopMotor();
    rightPivotMotor.stopMotor();
  }

  public void intiSendable(SendableBuilder builder){
    super.initSendable(builder);

    builder.addDoubleProperty("Left Pivot Speed", () -> leftPivotMotor.get(), null);
    builder.addDoubleProperty("Right Pivot Speed", () -> rightPivotMotor.get(), null); 
  }
}