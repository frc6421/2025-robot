// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CageIntakeSubsystem extends SubsystemBase {

  private final SparkMax cageIntakeMotor;

  private final SparkMaxConfig cageIntakeMotorConfig;

  public static class CageIntakeConstants {

    private static final int CAGE_INTAKE_MOTOR_ID = 52;

    private static final int CAGE_INTAKE_CURRENT_LIMIT = 150; // TODO: Update Numbers

    public static final double CAGE_INTAKE_SPEED = 1; // TODO: Update Numbers

    public static final double CAGE_STALL_LIMIT = 50;

  }

  /** Creates a new CilmbSubsystem. */
  public CageIntakeSubsystem() {
    cageIntakeMotor = new SparkMax(CageIntakeConstants.CAGE_INTAKE_MOTOR_ID, MotorType.kBrushless);

    cageIntakeMotorConfig = new SparkMaxConfig();
    cageIntakeMotorConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CageIntakeConstants.CAGE_INTAKE_CURRENT_LIMIT)
        .inverted(true);

    cageIntakeMotor.configure(cageIntakeMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Run Intake at set duty cycle.
   * </p>
   * 02/07/2025: Set to 0.5 or 50%.
   */
  public Command cageIntakeInSpeedCommand() {
    return this.run(() -> cageIntakeMotor.set(CageIntakeConstants.CAGE_INTAKE_SPEED));
  }

  /**
   * Determine if we have the cage in the intake based on the motors output
   * current</p>
   * TODO: Should there be a filter in case of spikes in current? 
   * 
   * @return true if current greater than stall limit.
   */
  public boolean haveCage() {
    return cageIntakeMotor.getOutputCurrent() > CageIntakeConstants.CAGE_STALL_LIMIT;
  }

  public Command stopCageIntakeMotorCommand() {
    return this.runOnce(() -> cageIntakeMotor.stopMotor());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("Current", () -> cageIntakeMotor.getOutputCurrent(), null);
  }
}
