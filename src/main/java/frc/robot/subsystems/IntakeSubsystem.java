// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intakeMotor;
  private TalonFXConfiguration intakeMotorConfig;

  public static class IntakeConstants {

    private static final int INTAKE_MOTOR_ID = 40;

    private static final int INTAKE_CURRENT_LIMIT = 200;

    private static final CurrentLimitsConfigs INTAKE_CURRENT_CONFIGS = new CurrentLimitsConfigs()
    .withStatorCurrentLimit(INTAKE_CURRENT_LIMIT)
    .withStatorCurrentLimitEnable(true);

    private static final double INTAKE_STALL_LIMIT = 60;
    // Reliable speed for grabbing the pieces
    public static final double INTAKE_IN_SPEED = 0.6;
    // Reliable speed for ejecting the pieces
    public static final double INTAKE_OUT_SPEED = -0.8; 

    private static final MotorOutputConfigs INTAKE_MOTOR_CONFIGS = new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Coast)
    .withInverted(InvertedValue.Clockwise_Positive);
  }

  /** Creates a new intakeSubsystem. */
  public IntakeSubsystem() {
    // Identifies the motor object 
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);

    RobotContainer.applyTalonConfigs(intakeMotor, new TalonFXConfiguration());

    intakeMotorConfig = new TalonFXConfiguration()
    .withCurrentLimits(IntakeConstants.INTAKE_CURRENT_CONFIGS)
    .withMotorOutput(IntakeConstants.INTAKE_MOTOR_CONFIGS);


     // Applies the configuration to the motor
    RobotContainer.applyTalonConfigs(intakeMotor, intakeMotorConfig);


    SmartDashboard.putData("Intake", this);
  }

  /**
   * @breif Sets the intake to the Intake In speed
   */

  public Command setIntakeSpeed(double output) {
    return runOnce(() -> intakeMotor.set(output));
  }

  /**
   * @breif Stops the motor
   */
  public Command stopIntake() {
    return runOnce(() -> intakeMotor.stopMotor());
  }

  public boolean haveCoral(){
    return intakeMotor.getStatorCurrent().getValueAsDouble()> IntakeConstants.INTAKE_STALL_LIMIT;
  }


public Command intakeCoral () {
  return run(() -> intakeMotor.set(IntakeConstants.INTAKE_IN_SPEED))
    .until(() -> haveCoral());
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Intake Speed", () -> intakeMotor.get(), null);
    builder.addDoubleProperty("Intake Current", () -> intakeMotor.getStatorCurrent().getValueAsDouble(), null);
  }

}
