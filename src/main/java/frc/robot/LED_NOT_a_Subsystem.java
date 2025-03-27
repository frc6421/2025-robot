// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
//TODO: Various things to keep in mind:
 * - Use the LEDs to display the auto that is currently selected
 * - Alliance color, mostly for diagnosing at home
 * - Planned Scoring position
 * - Notice to Human Player for coral
 * - More if anyone can think of them
 */
package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.Map;
import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LED_NOT_a_Subsystem.LEDConstants.LED_MODES;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorConstants;

public class LED_NOT_a_Subsystem extends SubsystemBase {
  public static class LEDConstants {
    public static final int PERIODIC_DELAY = 1;
    public static final int NUMBER_OF_LEDS = 100;
    public static final int[] WHITE = {255,255,255}, 
    PINK = {255,192,203}, CORAL = {255,127,80}, HOT_PINK = {255,105,180},
    RED = {255,0,0}, GREEN = {0,255,0}, BLUE = {0,0,255},
    VISION_BLUE = {50,50,255}, ALLIANCE_BLUE = {0,102,179}, ALIANCE_RED = {237,28,36},
    SEA_BLUE = {0,80,255}, FOAM_WHITE = {190,220,255}, SKY_BLUE = {150,150,255},
    GYRO_GREEN = {25,255,25}, ALGAE_GREEN = {80,255,100},
    OFF = {0,0,0},
    VISION_BACK_LEFT_COLOR = {0,0,255}, VISION_BACK_RIGHT_COLOR = {255,0,0};
    public static enum LED_MODES{
      RAINBOW,
      SNAKING_RAINBOW,
      COLLISION,
      WAVE,
    };
    private static int[] ELEVATOR_TARGET_COLOR = {0,255,0}, ELEVATOR_CURRENT_POS_COLOR = {0,0,255};
    private static int TRAILING_BRIGHTNESS = 7;//How long the brightness chain lasts. Larger is a smaller trail, smaller is a larger trail
  }
  private static AddressableLED led;//Led Strip
  private static AddressableLEDBuffer ledBuffer;//Buffer used before displaying to the strip
  private static int flickerDelay = 1;
  private static int[] patternColor = LEDConstants.OFF;
  private static int previousColorLimit;//Store the color limit for rainbow. When the rainbow is called during periodic, this value is used to store the 
  //previous position that the strip was at.
  private static int previousSnakingLED;//Stores the LED that was last at. Useful in making sure there are no loop overruns
  private static int RIO_PIN = 0;//TODO: Determine which pin of the Rio we are going to use the LEDs on
  private static int currentRobotCycle;//Current amount of waited robot cycles
  private static LED_MODES selectedDisabledPattern;
  private static double waveStep = Math.PI / 4;//The position of the sine wave, in radians.
  private static int waveHeight;
  //Stars for the disabled pattern
  private static int[] starPositions = {0,0,0,0,0};
  private static boolean starBool = false;
  private static int starAmount;
  //Sky brightness for the wave pattern.
  private static double waveSkyBrightness;
  private static double starBrightness = 0;
  

  /** Creates a new LEDSubsystem. */
  public LED_NOT_a_Subsystem() {
    //Setting the pin of the Rio that the LED strip is on
    led = new AddressableLED(RIO_PIN);
    //Creating the buffer that is used for setting the data, with the number of LED's, plus the trailing brightness
    ledBuffer = new AddressableLEDBuffer(LEDConstants.NUMBER_OF_LEDS);
    ledBuffer.setRGB(0,0,0,0);
    //Setting the length of the LED strip
    led.setLength(ledBuffer.getLength());
    //Setting the LEDs to whatever was in the buffer
    led.setData(ledBuffer);
    //Starting the LEDs
    led.start();
  }

  /**
   * LED Pattern when the Gyro is reset
   */
  public Command gyroReset(){
    setLED(LEDConstants.GYRO_GREEN);
    return run(()->flicker(4)).deadlineFor(new WaitCommand(0.02)).andThen(off());
  }

  /**
   * Sets the LED's to the Score color, White, for coral
   */
  public Command LEDScore(){
    return runOnce(() -> setLED(LEDConstants.WHITE));
  }

  /**
   * Sets the LED's to track the elevators position
   * @param  elevatorTarget The target of the elevator, given from the chooser
   * @param  elevator  The elevator subsystem object, given in the RobotContainer
   */
  public Command elevatorLEDPosition(double elevatorTarget, ElevatorSubsystem elevator){
    return run(() -> setElevatorLEDPosition(elevatorTarget, elevator.getElevatorRotations()));
  }

  /**
   * Sets the LED's to a specific color, but now with new Command Flavor and spicy timeouts!!
   * @param color  The color to set to
   * @param command  The Command to wait until is finished
   */
  public Command setLED(int color[], Command command){
    return run(() -> setLED(color)).until(() -> command.isFinished());
  }
  /**
   * Sets the LED's to a specific color, but now with new Command Flavor!
   * @param color  The color to set to
   * @param isACommand  A simple boolean that is used to select between the different setLED methods. Value does
   * not matter at all.
   */
  public Command setLED(int color[], boolean isACommand){
    return runOnce(() -> setLED(color));
  }

  /**
   * Turns the LEDs off
   */
  public Command off(){
    return runOnce(() -> setLED(LEDConstants.OFF));
  }


  /**
   * Sets the LED's to a specific color
   * @param color  The color to set to. 
   */
  public static void setLED(int color[]){
    //Creates a solid LED patern with the color given by the array, and applies it to the buffer
    LEDPattern.solid(new Color(color[0],color[1],color[2])).applyTo(ledBuffer); 
    led.setData(ledBuffer);
    patternColor = color;
  }
  /**
   * Mixes colors together. Useful if several things happen at once, but only one LED strip
   * @param newColor  The color to add.
   */
  public static void addLED(int newColor[]){
    for(int i = 0; i < 3; i++){
      if(newColor[i] + patternColor[i] >= 255) patternColor[i] = 255;
      else patternColor[i] += newColor[i];
    }
  }
  /**
   * Mixes colors together. Useful if several things happen at once, but only one LED strip
   * @param newColor  The color to subtract.
   */
  public static void subtractLED(int newColor[]){
    for(int i = 0; i < 3; i++){
      if(newColor[i] - patternColor[i] <= 0) patternColor[i] = 0;
      else patternColor[i] -= newColor[i];
    }
  }

  /**
   * Sets the LEDs to follow a pattern. The colors are set by setPatternColor and setBackgroundColor. 
   * This should be called in periodic, more realistically while disabled, since it can take away from the 
   * Rio processing.
   * @param mode  The LED mode to set to
   */
  public static void setLED(frc.robot.LED_NOT_a_Subsystem.LEDConstants.LED_MODES mode){
    switch(mode){
      case RAINBOW: 
        if(currentRobotCycle == LEDConstants.PERIODIC_DELAY){
          //if(previousColorLimit >= 1.5 * LEDConstants.NUMBER_OF_LEDS) previousColorLimit = 0;
          //Loops over the LEDs in the strip
          for(int LEDNum = 0; LEDNum < LEDConstants.NUMBER_OF_LEDS; LEDNum++){
            int wheelPos = previousColorLimit + LEDNum;
            //Creates a new color that gets from the rainbowColor function, taking in the 
            //color limit and the current LED 
            int color[] = rainbowColor(wheelPos);
            //Update buffer
            ledBuffer.setRGB(LEDNum, color[0], color[1], color[2]);
          }
          //Display the color
          led.setData(ledBuffer);
          previousColorLimit++;
          currentRobotCycle = 0;
        }else currentRobotCycle++;
      break;

      case SNAKING_RAINBOW:
        //Checking if it is at the number of delay cycles
        if(currentRobotCycle == LEDConstants.PERIODIC_DELAY){
          if(previousColorLimit > (LEDConstants.NUMBER_OF_LEDS + LEDConstants.TRAILING_BRIGHTNESS) * 2) previousColorLimit = 0;
          //Resets the previous snaking LED
          if(previousSnakingLED >= (LEDConstants.NUMBER_OF_LEDS + LEDConstants.TRAILING_BRIGHTNESS * 2) + 
            ((LEDConstants.NUMBER_OF_LEDS + LEDConstants.TRAILING_BRIGHTNESS * 2) * (LEDConstants.TRAILING_BRIGHTNESS * 2 / 100))){
            previousSnakingLED = 0;
          }
          //Gets the color, taking in the Previous Color Limit and the Previous Snaking LED
          int color[] = rainbowColor(previousColorLimit + previousSnakingLED);
          //Updating the buffer with the color for the head LED
          ledBuffer.setRGB(previousSnakingLED, color[0], color[1], color[2]);
          //Creating the tail
          for(int LEDNum = 0; LEDNum < previousSnakingLED; LEDNum++){
            //Makes a varaible to hold the values for getting the color, only if modificatios want to be made
            int wheelPos = previousColorLimit + LEDNum;
            //Getting the trailing color
            color = rainbowColor(wheelPos);
            //Decreasing the brightness of the color, looping over each part of it.
            for(int i = 0; i < 3; i++){
              color[i] = (color[i] - ((previousSnakingLED - LEDNum) * LEDConstants.TRAILING_BRIGHTNESS * 2) < 0 ? 0 
              : color[i] - ((previousSnakingLED - LEDNum) * LEDConstants.TRAILING_BRIGHTNESS * 2));
            }
            //Setting that brightness change
            ledBuffer.setRGB(LEDNum, color[0], color[1], color[2]);
          }
          //Increment the color limit, snaking LED, and sets the data to the strip.
          previousColorLimit++;
          previousSnakingLED++;
          led.setData(ledBuffer);
        }else currentRobotCycle++;
      break;

      case COLLISION: 
        if(currentRobotCycle == LEDConstants.PERIODIC_DELAY){
          if(previousColorLimit == 255) previousColorLimit = 0;
          else previousColorLimit++;
          //Loops over the LEDs in the strip
          for(int LEDNum = 0; LEDNum < LEDConstants.NUMBER_OF_LEDS; LEDNum++){
            int wheelPos = previousColorLimit + LEDNum;
            //Creates a new color that gets from the rainbowColor function, taking in the 
            //color limit and the current LED 
            int color[] = rainbowColor(wheelPos);
            int trailing = previousSnakingLED * (LEDConstants.TRAILING_BRIGHTNESS);
            //Adding the tail by decreasing the "brightness"
            color[0] = (trailing < 0 ? 0 : color[0] - trailing);
            color[1] = (trailing < 0 ? 0 : color[1] - trailing);
            color[2] = (trailing < 0 ? 0 : color[2] - trailing);
            //Update buffer
            ledBuffer.setRGB(LEDNum, color[0], color[1], color[2]);
          }
          led.setData(ledBuffer);
          currentRobotCycle = 0;
          previousSnakingLED++;
        }else currentRobotCycle++;
      break;

      case WAVE:
        if(waveStep == 2*Math.PI) waveStep = 0;
        //The acutal height of the wave
        long position = Math.round(4 * Math.sin(waveStep) + waveHeight) + Math.round(8 * Math.sin(waveStep / 16));
        waveSkyBrightness = (0.5 * Math.sin(waveStep / 16 + Math.PI / 4)) - 0.5;
        for(int LEDNum = 0; LEDNum < LEDConstants.NUMBER_OF_LEDS; LEDNum++){
          //Setting the wave
          if(LEDNum <= position) ledBuffer.setRGB(LEDNum, LEDConstants.SEA_BLUE[0], LEDConstants.SEA_BLUE[1], LEDConstants.SEA_BLUE[2]);
          //Setting the layer of foam on top
          if(LEDNum == position + 1 || LEDNum == position + 2){
            ledBuffer.setRGB(LEDNum, LEDConstants.FOAM_WHITE[0], LEDConstants.FOAM_WHITE[1], LEDConstants.FOAM_WHITE[2]);
          }
          if(LEDNum > position + 2){ ledBuffer.setRGB(LEDNum, 
            (int)(LEDConstants.SKY_BLUE[0] + LEDConstants.SKY_BLUE[0] * waveSkyBrightness), 
            (int)(LEDConstants.SKY_BLUE[1] + LEDConstants.SKY_BLUE[1] * waveSkyBrightness), 
            (int)(LEDConstants.SKY_BLUE[2] + LEDConstants.SKY_BLUE[2] * waveSkyBrightness));
          }
          if(waveSkyBrightness < 0.5 * Math.sin(Math.PI / 8) - 0.5){
            if(!starBool){
              starAmount = new Random().nextInt(1,5);
              for(int i = 0; i < starAmount; i++){
                //Generates the star position above the maximum of each sine wave for position.
                starPositions[i] = new Random().nextInt(
                  (int) Math.round(4 * Math.sin(Math.PI / 2) + waveHeight + 10 + (8 * Math.sin(Math.PI / 32))), 
                  LEDConstants.NUMBER_OF_LEDS);
              }
              starBool = true;
            }else{
              if(0.5 / 16 * Math.cos(waveStep / 16 + Math.PI / 4) < 0 && starBrightness <= 254) starBrightness += 0.005;
              if(0.5 / 16 * Math.cos(waveStep / 16 + Math.PI / 4) > 0 && starBrightness >= -1) starBrightness -= 0.002;
              int[] color = {
                (int)(starBrightness + LEDConstants.SKY_BLUE[0] + LEDConstants.SKY_BLUE[0] * waveSkyBrightness),
                (int)(starBrightness + LEDConstants.SKY_BLUE[1] + LEDConstants.SKY_BLUE[1] * waveSkyBrightness),
                (int)(starBrightness + LEDConstants.SKY_BLUE[2] + LEDConstants.SKY_BLUE[2] * waveSkyBrightness)
              };
              for(int i = 0; i < 3; i++){
                if(color[i] > 255) color[i] = 255;
                if(color[i] < 0) color[i] = 0;
              }
              for(int i = 0; i < starAmount; i++){
                //Setting the stars
                ledBuffer.setRGB(starPositions[i], color[0], color[1], color[2]);
              }
            }
          }else{
            starBool = false;
            starBrightness = 0;
          }
        }
        waveStep += Math.PI / 128;
        led.setData(ledBuffer);
      break;
      default: break;
    }
  }

  /**
   * Flickers the LED's every few robot cycles
   */
  public static void flicker(){
    if(currentRobotCycle == flickerDelay){
      LEDPattern.solid(new Color(patternColor[0], patternColor[1], patternColor[2])).applyTo(ledBuffer);
      led.setData(ledBuffer);
      currentRobotCycle = 0;
    }else{
      LEDPattern.solid(new Color(0,0,0)).applyTo(ledBuffer);
      led.setData(ledBuffer);
      currentRobotCycle++;
    }
  }

  public static void flicker(int delay){
    flickerDelay = delay;
    if(currentRobotCycle == flickerDelay){
      LEDPattern.solid(new Color(patternColor[0], patternColor[1], patternColor[2])).applyTo(ledBuffer);
      led.setData(ledBuffer);
      currentRobotCycle = 0;
    }else{
      LEDPattern.solid(new Color(0,0,0)).applyTo(ledBuffer);
      led.setData(ledBuffer);
      currentRobotCycle++;
    }
  }
  /**
   * Takes in the current position of the Elevator and uses it to display where the elevator will be going
   * @param elevatorTarget  The Distance object of the Elevator position
   * @param currentElevatorHeight  The current height of the elevator, given in inches
   */
  public static void setElevatorLEDPosition(double elevatorTarget, double currentElevatorHeight){
    int elevatorUpperTarget, elevatorLowerTarget;
    //Setting the lower position to be the next-lowest set scoring position
    //L1 position, should be the lowest 
    if(elevatorTarget <= ElevatorConstants.L1_POSITION.in(Inches)){
      elevatorUpperTarget = LEDConstants.NUMBER_OF_LEDS * 1 / 5;
      elevatorLowerTarget = LEDConstants.NUMBER_OF_LEDS * 0 / 5;
    }else if(elevatorTarget <= ElevatorConstants.L2_POSITION.in(Inches)){
      elevatorUpperTarget = LEDConstants.NUMBER_OF_LEDS * 2 / 5;
      elevatorLowerTarget = LEDConstants.NUMBER_OF_LEDS * 1 / 5;
    }else if(elevatorTarget <= ElevatorConstants.STATION_POSITION.in(Inches)){
      elevatorUpperTarget = LEDConstants.NUMBER_OF_LEDS * 3 / 5;
      elevatorLowerTarget = LEDConstants.NUMBER_OF_LEDS * 2 / 5;
    }else if(elevatorTarget <= ElevatorConstants.L3_POSITION.in(Inches)){
      elevatorUpperTarget = LEDConstants.NUMBER_OF_LEDS * 4 / 5;
      elevatorLowerTarget = LEDConstants.NUMBER_OF_LEDS * 3 / 5;
    }else{
      elevatorUpperTarget = LEDConstants.NUMBER_OF_LEDS * 5 / 5;
      elevatorLowerTarget = LEDConstants.NUMBER_OF_LEDS * 4 / 5;
    }
    //Using the lower elevator position, black, the max elevator position at that point, and the pattern color in RGB.
    LEDPattern.steps(Map.of(currentElevatorHeight, Color.kBlack, 0.0, new Color(
      LEDConstants.ELEVATOR_CURRENT_POS_COLOR[0],
      LEDConstants.ELEVATOR_CURRENT_POS_COLOR[1],
      LEDConstants.ELEVATOR_CURRENT_POS_COLOR[2]
      )
    )).applyTo(ledBuffer);//Applying the pattern to the buffer
    //Looping for the target LED colors
    for(int i = elevatorLowerTarget; i < elevatorUpperTarget; i++){
      ledBuffer.setRGB(i, LEDConstants.ELEVATOR_TARGET_COLOR[0], LEDConstants.ELEVATOR_TARGET_COLOR[1], LEDConstants.ELEVATOR_TARGET_COLOR[2]);
    }
    led.setData(ledBuffer);//Update the strip
  }

  public static LED_MODES getDisabledPattern(){
    return selectedDisabledPattern;
  }

  /**
   * Sets the various things when the robot is disabled. Sets the pattern and colors to random
   * each disabled period
   */
  public static void setDisabledPattern(){
    //Random Object
    Random random = new Random();
    //There are six different patterns
    switch(random.nextInt(4)){
      case 0: selectedDisabledPattern = LED_MODES.RAINBOW; break;
      case 1: 
        selectedDisabledPattern = LED_MODES.SNAKING_RAINBOW; 
        ledBuffer = new AddressableLEDBuffer(LEDConstants.NUMBER_OF_LEDS + LEDConstants.TRAILING_BRIGHTNESS * 2);
        ledBuffer.setRGB(0,0,0,0);
        //Setting the length of the LED strip
        led.setLength(ledBuffer.getLength());break;
      case 2: selectedDisabledPattern = LED_MODES.COLLISION; break;
      case 3: 
        selectedDisabledPattern = LED_MODES.WAVE; 
        waveHeight = random.nextInt(20, Math.round(LEDConstants.NUMBER_OF_LEDS / 2));
      break;
    }
    LED_NOT_a_Subsystem.setLED(selectedDisabledPattern);
  }
  /**
   * Resets the length of the LED strip to the normal length. This should only be used after periodic
   */
  public static void resetLength(){ 
    ledBuffer = new AddressableLEDBuffer(LEDConstants.NUMBER_OF_LEDS); 
    led.setLength(ledBuffer.getLength());
  }


  //Used for determine the color to be assigned to an LED based on its position on the strip
  //and the current color limit
  private static int[] rainbowColor(int wheelPos){
    int r = 0, g = 0, b = 0;
    while(wheelPos / 255 > 0) wheelPos -= 255;
    if(wheelPos < 85) {
      r = wheelPos * 3;
      g = 255 - wheelPos * 3;
      b = 0;
    } 
    else if(wheelPos < 170) {
      wheelPos -= 85;
      r = 255 - wheelPos * 3;
      g = 0;
      b = wheelPos * 3;
    } 
    else{
      wheelPos -= 170;
      r = 0;
      g = wheelPos * 3;
      b = 255 - wheelPos * 3;
    }
    int[] values = {r,g,b};
    return values;
  }
  
  @Override
  public void periodic() {
    //Nothing in here...
  }
}


