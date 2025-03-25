// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristCommand extends Command {
  //TODO: Update with the values we desire
  private final double DELTA_TIME = 0.02;
  private final double WRIST_KG = 0.4;
  private final double WRIST_KS = 0.1;
  public static final double WRIST_KV = 0.1;//In Volts per radians per second
  private final double WRIST_MAX_VELOCITY = 300;//In rad/sec
  private final double WRIST_MAX_ACCELERATION = 750;//In rad/sec^2

  private final WristSubsystem wrist;//Empty Wrist Object
  private Timer timer = new Timer();//Set up the timer 

  private final TrapezoidProfile.Constraints wristConstraints = new TrapezoidProfile.Constraints(
    WRIST_MAX_VELOCITY, WRIST_MAX_ACCELERATION);
  
  private TrapezoidProfile.State wristInitial = new TrapezoidProfile.State();//Initial Wrist Position
  private TrapezoidProfile.State wristGoal = new TrapezoidProfile.State();//Creates the empty Wrist Goal
  private TrapezoidProfile.State wristCurrent = new TrapezoidProfile.State();
  private TrapezoidProfile.State wristNext = new TrapezoidProfile.State();//Creates the empty Wrist Set point
 
  private TrapezoidProfile wristProfile = new TrapezoidProfile(wristConstraints);//Creating the Profile
 
  private double goToPos;//Position to set the Wrist to 
  private final ArmFeedforward armFeedForward = new ArmFeedforward(WRIST_KS, WRIST_KG, WRIST_KV);

  private double currentTime;

  /** Creates a new WristCommand. */
  public WristCommand(WristSubsystem wristSubsystem, double position) {
    addRequirements(wristSubsystem);
    wrist = wristSubsystem;//Update the Wrist Object with the Wrist Subsystem
    goToPos = position;//Setting the Position to go to
    SmartDashboard.putData("Wrist Tuning", this);
  }

  /**
   * Called when the Command is initally scheduled.
   */
  @Override
  public void initialize(){
    timer.reset();//Resets the Timer
    wristInitial = new TrapezoidProfile.State(wrist.getWristEncoderPosition(),0);//Sets the inital Wrist Position
    wristGoal = new TrapezoidProfile.State(goToPos, 0);//Sets the position
    timer.start();//Starts the timer
  }


  /**
   * Called when the Scheduler runs while the Command is scheduled
   */
  @Override
  public void execute(){
    currentTime = timer.get();//Getting the current time, to somewhat prevent the two different calculations from 
    //differing
    wristCurrent = wristProfile.calculate(currentTime, wristInitial, wristGoal);//Calculates the current wrist position
    wristNext = wristProfile.calculate(currentTime + DELTA_TIME, wristInitial, wristGoal);//Calculates the next wrist position
    System.out.println("Position: " + wristNext.position);
    //System.out.println(wristProfile.timeLeftUntil(goToPos));

    wrist.setAngle(wristNext.position, armFeedForward.calculateWithVelocities(
      Math.toRadians(wrist.getWristEncoderPosition()), 
      Math.toRadians(wristCurrent.velocity), 
      Math.toRadians(wristNext.velocity))
      );//Setting the angle of the motor
  }


  /**
   * Called when the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted){
    wrist.setAngle(wristNext.position, 0);
  }


  /**
   * Determines if the command should end
   * @return  true when the command *should* end
   */
  @Override
  public boolean isFinished(){
    return wristProfile.isFinished(currentTime);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Current Velocity", ()->this.wristCurrent.velocity, null);
    builder.addDoubleProperty("Next Velocity", ()->this.wristNext.velocity, null);
    builder.addDoubleProperty("Current Position", ()->this.wristCurrent.position, null);
    builder.addDoubleProperty("Next Position", ()->this.wristNext.position, null);
  }
}