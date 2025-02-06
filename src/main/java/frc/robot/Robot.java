// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Random;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LED_NOT_a_Subsystem.LEDConstants;
import frc.robot.LED_NOT_a_Subsystem.LEDConstants.LED_MODES;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private static LEDConstants.LED_MODES selectedMode;
  private static int periodicDelay;
  //Putting in some random colors so the loop to select the colors doesn't just exit immediatly
  private static int[][] patternColor = { LEDConstants.WHITE, LEDConstants.PINK };
  private static final int PATTERN_DELAY = 50;//TODO: Find a good value for this, like the rainbow patterns and such

  public Robot() {
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }
/***********LEDs IN PERIODIC************/
  @Override
  public void disabledInit() {
    //Random Object
    Random random = new Random();
    periodicDelay = random.nextInt(25 + PATTERN_DELAY);
    LED_NOT_a_Subsystem.setPeriodicDelay(periodicDelay);
    //There are six different patterns
    switch(random.nextInt(4)){
      case 0: selectedMode = LED_MODES.RAINBOW; break;
      case 1: selectedMode = LED_MODES.SNAKING_RAINBOW; break;
      case 2: selectedMode = LED_MODES.SLOW_FILL; break;
      case 3: selectedMode = LED_MODES.RANDOM_FLICKER; break;
      default: 
        System.out.println();
        System.out.println("Someone forgot how to math....");
        System.out.println("It was me. I forgot.");
        System.out.println();
      break;
    }
    LED_NOT_a_Subsystem.setLED(selectedMode);
    if(selectedMode == LED_MODES.SLOW_FILL || selectedMode == LED_MODES.RANDOM_FLICKER){
      boolean pass = false;
      while(patternColor[0] != patternColor[1] && pass){
        for(int i = 0; i < 2; i++){
          switch(random.nextInt(7)){
            case 0: patternColor[i] = LEDConstants.WHITE; break;
            case 1: patternColor[i] = LEDConstants.PINK; break;
            case 2: patternColor[i] = LEDConstants.CORAL; break;
            case 3: patternColor[i] = LEDConstants.HOT_PINK; break;
            case 4: patternColor[i] = LEDConstants.RED; break;
            case 5: patternColor[i] = LEDConstants.GREEN; break;
            case 6: patternColor[i] = LEDConstants.BLUE; break;
            default: 
              System.out.println();
              System.out.println("Who can't math again?");
              System.out.println("Me. HOW CAN I NOT DO THIS???!?!?!");
              System.out.println();
            break;
          }
        }
        //Checking to make sure that they aren't some atrocious mixture
        if((patternColor[0] == LEDConstants.BLUE && patternColor[1] == LEDConstants.GREEN) 
        || (patternColor[0] == LEDConstants.GREEN && patternColor[1] == LEDConstants.BLUE)) pass = false;
        else pass = true;
        if((patternColor[0] == LEDConstants.PINK && patternColor[1] == LEDConstants.HOT_PINK) 
        || (patternColor[0] == LEDConstants.HOT_PINK && patternColor[1] == LEDConstants.PINK)) pass = false;
        else pass = true;
      }
      //Setting the stuff for the LED's
      LED_NOT_a_Subsystem.setBackgroundColor(patternColor[0]);
      LED_NOT_a_Subsystem.setPatternColor(patternColor[1]);
    }else LED_NOT_a_Subsystem.setPeriodicDelay(PATTERN_DELAY);
  }

  @Override
  public void disabledPeriodic() {
    //Updates the strip(s). And by that I mean it runs the method and changes the colors on the strip. Even if it runs each robot cycle, the set delay should do wonders
    LED_NOT_a_Subsystem.setLED(selectedMode);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
