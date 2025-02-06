// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;

public class CageIntakeSubsystem extends SubsystemBase {

  private final SparkMax cageIntakeMotor;

  private final SparkMaxConfig cageIntakeMotorConfig;

  public static class CageIntakeConstants {

    private static final int CAGE_INTAKE_MOTOR_ID = 52;

    private static final int CAGE_INTAKE_CURRENT_LIMIT = 80; // TODO: Update Numbers

    public static final double CAGE_INTAKE_SPEED = 0.5; // TODO: Update Numbers

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

    SmartDashboard.putNumber("Current", cageIntakeMotor.getOutputCurrent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command cageIntakeInSpeedCommand() {
    return this.run(() -> cageIntakeMotor.set(CageIntakeConstants.CAGE_INTAKE_SPEED));
  }

    public boolean haveCage(){
    return cageIntakeMotor.getOutputCurrent()> CageIntakeConstants.CAGE_STALL_LIMIT;
  }
 
  public Command stopCageIntakeMotorCommand() {
    return this.runOnce(() -> cageIntakeMotor.stopMotor());
  }

}
