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

import java.util.Random;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LED_NOT_a_Subsystem.LEDConstants.LED_MODES;
//import frc.robot.subsystems.ElevatorSubsytem;

public class LED_NOT_a_Subsystem extends SubsystemBase {
  public static class LEDConstants {
    public static final int PERIODIC_DELAY = 2;
    public static final int NUMBER_OF_LEDS = 100;
    public static final int[] WHITE = {255,255,255}, 
    PINK = {255,192,203}, CORAL = {255,127,80}, HOT_PINK = {255,105,180},
    RED = {255,0,0}, GREEN = {0,255,0}, BLUE = {0,0,255},
    VISION_BLUE = {50,50,255}, ALLIANCE_BLUE = {0,102,179}, ALIANCE_RED = {237,28,36};
    public static enum LED_MODES{
      RAINBOW,
      FLICKER,
      SNAKING_RAINBOW,
      HIGHLIGHT,
      COLLISION,
    }
    //TODO: Set the desired color for the Elevator target
    private static int[] ELEVATOR_TARGET_COLOR = {0,0,0};
    private static int TRAILING_BRIGHTNESS = 10;//How long the brightness chain lasts. Larger is a smaller trail, smaller is a larger trail
  }

  private static AddressableLED led;//Led Strip
  private static AddressableLEDBuffer ledBuffer;//Buffer used before displaying to the strip

  private static int[] patternColor = LEDConstants.WHITE;

  private static int previousColorLimit;//Store the color limit for rainbow. When the rainbow is called during periodic, this value is used to store the 
  //previous position that the strip was at.
  private static int[] previousSlowFillLED = {LEDConstants.NUMBER_OF_LEDS,0};//Stores the LED that it was last used.
  private static int previousSnakingLED;//Stores the LED that was last at. Useful in making sure there are no loop overruns

  private static double elevatorLowerPos;//Lower Elevator position as a percent of the strip
  private static double elevatorUpperPos;//Maximum Elevator position for a given scoring position as a percent
  private static Distance setElevatorPosition;//Set point of the elevator

  private static int RIO_PIN = 0;//TODO: Determine which pin of the Rio we are going to use the LEDs on

  private static int currentRobotCycle;//Current amount of waited robot cycles

  private static int snakingOverflow;

  private static LED_MODES selectedDisabledPattern;


  
    /** Creates a new LEDSubsystem. */
    public LED_NOT_a_Subsystem() {
      //Setting the pin of the Rio that the LED strip is on
      led = new AddressableLED(RIO_PIN);
      //Creating the buffer that is used for setting the data
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
     * @breif   Sets the LEDS to a specific color
     * @param color  The color to set to. 
     */
    public void setLED(int color[]){
      //Creates a solid LED patern with the color given by the array, and applies it to the buffer
      LEDPattern.solid(new Color(color[0],color[1],color[2])).applyTo(ledBuffer); 
      led.setData(ledBuffer);
    }
  
    /**
     * @breif   Sets the LEDs to follow a pattern. The colors are set by setPatternColor and setBackgroundColor. 
     * This should be called in periodic, more realistically while disabled, since it can take away from the 
     * Rio processing.
     * @param mode  The LED mode to set to
     */
    public static void setLED(frc.robot.LED_NOT_a_Subsystem.LEDConstants.LED_MODES mode){
      switch(mode){
        case RAINBOW: 
          if(currentRobotCycle == LEDConstants.PERIODIC_DELAY){
            if(previousColorLimit == 255) previousColorLimit = 0;
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
          if(currentRobotCycle == LEDConstants.PERIODIC_DELAY){
            if(previousColorLimit == 255) previousColorLimit = 0;
            if(previousSnakingLED == LEDConstants.NUMBER_OF_LEDS + (LEDConstants.NUMBER_OF_LEDS * (LEDConstants.TRAILING_BRIGHTNESS / 100))){
              previousSnakingLED = 0;
              snakingOverflow++;
            }
            int color[] = rainbowColor(previousColorLimit + previousSnakingLED);
            ledBuffer.setRGB(previousSnakingLED, color[0], color[1], color[2]);
            for(int LEDNum = 0; LEDNum < previousSnakingLED; LEDNum++){
              int wheelPos = previousColorLimit + LEDNum;
              color = rainbowColor(wheelPos);
              for(int i = 0; i < 3; i++){
                color[i] = (color[i] - ((previousSnakingLED - LEDNum) * LEDConstants.TRAILING_BRIGHTNESS) < 0 ? 0 : color[i] - ((previousSnakingLED - LEDNum) * LEDConstants.TRAILING_BRIGHTNESS));
              }
              ledBuffer.setRGB(LEDNum, color[0], color[1], color[2]);
            }
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
  
        case FLICKER:
          if(currentRobotCycle == LEDConstants.PERIODIC_DELAY){
            LEDPattern.solid(new Color(patternColor[0], patternColor[1], patternColor[2])).applyTo(ledBuffer);
            led.setData(ledBuffer);
            currentRobotCycle = 0;
          }else{
            LEDPattern.solid(new Color(0,0,0)).applyTo(ledBuffer);
            led.setData(ledBuffer);
            currentRobotCycle++;
          }
        break;
  
        case HIGHLIGHT:
        /* 
          //Using the lower elevator position, black, the max elevator position at that point, and the pattern color in RGB.
          LEDPattern.steps(Map.of(elevatorLowerPos, Color.kBlack, elevatorUpperPos, new Color(patternColor[0], patternColor[1], patternColor[2])))
          .applyTo(ledBuffer);//Applying the pattern to the buffer
          int upperLimit, lowerLimit;
          //Calculating the range of LEDs to set to give the set point of the elevator
          if(setElevatorPosition == ElevatorConstants.L1_POSITION){
            upperLimit = LEDConstants.NUMBER_OF_LEDS * 5 / 5;
            lowerLimit = LEDConstants.NUMBER_OF_LEDS * 4 / 5;
          }else if(setElevatorPosition == ElevatorConstants.L2_POSITION){
            upperLimit = LEDConstants.NUMBER_OF_LEDS * 4 / 5;
            lowerLimit = LEDConstants.NUMBER_OF_LEDS * 3 / 5;
          }else if(setElevatorPosition == ElevatorConstants.STATION_POSITION){
            upperLimit = LEDConstants.NUMBER_OF_LEDS * 3 / 5;
            lowerLimit = LEDConstants.NUMBER_OF_LEDS * 2 / 5;
          }else if(setElevatorPosition == ElevatorConstants.L3_POSITION){
            upperLimit = LEDConstants.NUMBER_OF_LEDS * 2 / 5;
            lowerLimit = LEDConstants.NUMBER_OF_LEDS * 1 / 5;
          }else if(setElevatorPosition == ElevatorConstants.L4_POSITION){
            upperLimit = LEDConstants.NUMBER_OF_LEDS * 1 / 5;
            lowerLimit = LEDConstants.NUMBER_OF_LEDS * 0 / 5;
          }
          //Looping for the target LED colors
          for(int i = upperLimit; i < lowerLimit; i++){
            ledBuffer.setRGB(i, LEDConstants.ELEVATOR_TARGET_COLOR[0], LEDConstants.ELEVATOR_TARGET_COLOR[1], LEDConstants.ELEVATOR_TARGET_COLOR[2]);
          }
          led.setData(ledBuffer);//Update the strip*/
        break;
        default: break;
      }
    }
  
    /**
     * @breif   Takes in the current position of the Elevator and uses it to display where the elevator will be going
     * @param position  The Distance object of the Elevator position
     * @param currentElevatorHeight  The current height of the elevator, given in meters
     *//* 
    public void setElevatorLEDPosition(Distance setPosition, double currentElevatorHeight){
      setElevatorPosition = setPosition;
      //Calculating the maximum of which to light the strip at
      elevatorUpperPos = (ElevatorConstants.MAX_HEIGHT - currentElevatorHeight) / ElevatorConstants.MAX_HEIGHT;
      //Setting the lower position to be the next-lowest set scoring position
      //L1 position, should be the lowest 
      if(currentElevatorHeight <= ElevatorConstants.L1_POSITION) elevatorLowerPos = 0.00;
      else if(currentElevatorHeight <= ElevatorConstants.L2_POSITION) elevatorLowerPos = 0.20;
      else if(currentElevatorHeight <= ElevatorConstants.STATION_POSITION) elevatorLowerPos = 0.40;
      else if(currentElevatorHeight <= ElevatorConstants.L3_POSITION) elevatorLowerPos = 0.60;
      else if(currentElevatorHeight <= ElevatorConstants.L4_POSITION) elevatorLowerPos = 0.80;
    }*/
  
    /**
     * @breif   Turns the LEDs off
     */
    public void off(){
      LEDPattern.solid(Color.kBlack).applyTo(ledBuffer);
      led.setData(ledBuffer);
    }
  
    public static LED_MODES getDisabledPattern(){
      return selectedDisabledPattern;
  }

  /**
   * @breif  Sets the various things when the robot is disabled. Sets the pattern and colors to random
   * each disabled period
   */
  public static void setDisabledPattern(){
    //Random Object
    Random random = new Random();
    //There are six different patterns
    switch(random.nextInt(3)){
      case 0: selectedDisabledPattern = LED_MODES.RAINBOW; break;
      case 1: selectedDisabledPattern = LED_MODES.SNAKING_RAINBOW; break;
      case 2: selectedDisabledPattern = LED_MODES.COLLISION; break;
    }
    LED_NOT_a_Subsystem.setLED(selectedDisabledPattern);
  }


  //Used for determine the color to be assigned to an LED based on its position on the strip
  //and the current color limit
  private static int[] rainbowColor(int wheelPos){
    int r = 0, g = 0, b = 0;
    if(wheelPos > 255) wheelPos -= 255;
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


