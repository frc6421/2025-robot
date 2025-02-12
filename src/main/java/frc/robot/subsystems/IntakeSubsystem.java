// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private SparkMax intakeMotor;
  private SparkMaxConfig intakeMotorConfig;

  public static class IntakeConstants {

    private static final int INTAKE_MOTOR_ID = 40;

    private static final int INTAKE_CURRENT_LIMIT = 80;

    private static final int INTAKE_STALL_LIMIT = 30;
    // Reliable speed for grabbing the pieces
    public static final double INTAKE_IN_SPEED = 0.5;
    // Reliable speed for ejecting the pieces
    public static final double INTAKE_OUT_SPEED = -0.5; 
  }

  /** Creates a new intakeSubsystem. */
  public IntakeSubsystem() {
    // Identifies the motor object as a Spark Max Controller
    intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    intakeMotorConfig = new SparkMaxConfig();
    // TODO: Do we need Brake when have coral?
    intakeMotorConfig.idleMode(IdleMode.kCoast);// Sets the motor to freely rotate
    intakeMotorConfig.smartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT);// Setting current limit
    intakeMotorConfig.inverted(true);// Inverts

    // Applies the configuration to the motor
    intakeMotor.configure(intakeMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    SmartDashboard.putData("Intake/Intake Speed", this);
  }

  /**
   * @breif Sets the intake to the Intake In speed
   */
  public void setIntakeInSpeed() {
    intakeMotor.set(IntakeConstants.INTAKE_IN_SPEED);
  }

  /**
   * @breif Sets the intake to the Intake Out speed
   */
  public void setIntakeOutSpeed() {
    intakeMotor.set(IntakeConstants.INTAKE_OUT_SPEED);
  }

  /**
   * @breif Sets the intake voltage
   * @param value Voltage to set
   */
  public void setIntakeVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  /**
   * @breif Stops the motor
   */
  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  public boolean haveCoral(){
    return intakeMotor.getOutputCurrent()> IntakeConstants.INTAKE_STALL_LIMIT;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intiSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Intake Speed", () -> intakeMotor.get(), null);
  }

}
