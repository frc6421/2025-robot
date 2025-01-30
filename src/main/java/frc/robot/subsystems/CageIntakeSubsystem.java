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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CageIntakeSubsystem extends SubsystemBase {

  private final SparkMax cageIntakeMotor;

  private final SparkMaxConfig cageIntakeMotorConfig;

  public static class CageIntakeConstants {


    private static final int CAGE_INTAKE_MOTOR_ID = 52;

    private static final int CAGE_INTAKE_CURRENT_LIMIT = 0; // TODO: Update Numbers

    // soft limits
    private static double CAGE_INTAKE_FORWARD_SOFT_LIMIT = 0; // TODO: Update Numbers
    private static double CAGE_INTAKE_REVERSE_SOFT_LIMIT = 0; // TODO: Update Numbers


    public static final int CAGE_INTAKE_IN_SPEED = 0; // TODO: Update Numbers
    public static final int CAGE_INTAKE_OUT_SPEED = 0; // TODO: Update Numbers

  }

  /** Creates a new CilmbSubsystem. */
  public CageIntakeSubsystem() {
    cageIntakeMotor = new SparkMax(CageIntakeConstants.CAGE_INTAKE_MOTOR_ID, MotorType.kBrushless);

    cageIntakeMotorConfig = new SparkMaxConfig();
    cageIntakeMotorConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CageIntakeConstants.CAGE_INTAKE_CURRENT_LIMIT)
        .inverted(true);

    cageIntakeMotorConfig.softLimit.forwardSoftLimit(CageIntakeConstants.CAGE_INTAKE_FORWARD_SOFT_LIMIT)
        .forwardSoftLimitEnabled(true);
    cageIntakeMotorConfig.softLimit.reverseSoftLimit(CageIntakeConstants.CAGE_INTAKE_REVERSE_SOFT_LIMIT)
        .forwardSoftLimitEnabled(true);

    cageIntakeMotor.configure(cageIntakeMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

        SmartDashboard.putData("Cage Intake", this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void cageIntakeInSpeed() {
    cageIntakeMotor.set(CageIntakeConstants.CAGE_INTAKE_IN_SPEED);
  }

  public void cageIntakeOutSpeed() {
    cageIntakeMotor.set(CageIntakeConstants.CAGE_INTAKE_OUT_SPEED);
  }

  public void stopCageIntakeMotor() {
    cageIntakeMotor.stopMotor();
  }

  public void intiSendable(SendableBuilder builder){
    super.initSendable(builder);

    builder.addDoubleProperty("Cage Intake Speed", () -> cageIntakeMotor.get(),null);
  }

}
