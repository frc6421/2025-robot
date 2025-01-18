// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {
  private SparkMax intakeMotor;
  private SparkMaxConfig intakeMotorConfig;

  private RelativeEncoder intakeEncoder; 
  
  public static class IntakeConstants{

    public static final int INTAKE_MOTOR_ID = 40;

    public static final int INTAKE_GEAR_RATIO = 1;    

    public static final int INTAKE_CURRENT_LIMIT = 0;

    public static final double INTAKE_IN_SPEED = 0;

    public static final double INTAKE_OUT_SPEED = 0;
  }

  /** Creates a new intakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    // intakeMotor.configure(null, SparkBase.ResetMode.kResetSafeParameters, null);

    intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig.smartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT);
    intakeMotorConfig.inverted(true);

    intakeEncoder = intakeMotor.getEncoder();

    intakeMotor.configure(intakeMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
  }

    // Sets the intakeMotor outputs
    public void setIntakeInSpeed(double value){
      intakeMotor.set(IntakeConstants.INTAKE_IN_SPEED);
    }
  
    public void setIntakeOutSpeed(double value){
      intakeMotor.set(IntakeConstants.INTAKE_OUT_SPEED);
    }
  
    public void setIntakeVoltage(double value){
      intakeMotor.setVoltage(0);
    }
  
    // stops intakeMotor
    public void stopIntake(){
      intakeMotor.stopMotor();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
