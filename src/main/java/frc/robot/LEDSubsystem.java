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

import java.util.Map;
import java.util.Optional;
import java.util.Random;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorSubsytem;

public class LEDSubsystem extends SubsystemBase {
  public static class LEDConstants {
    public static int NUMBER_OF_LEDS = 100;
    public static int[] WHITE = {255,255,255}, 
    PINK = {255,192,203}, CORAL = {255,127,80}, HOT_PINK = {255,105,180},
    RED = {255,0,0}, GREEN = {0,255,0}, VISION_BLUE = {0,0,255},
    ALLIANCE_BLUE = {0,102,179}, ALIANCE_RED = {237,28,36};
    public static enum LED_MODES{
      RAINBOW,
      SLOW_FILL,
      RANDOM_FLICKER,
      FLICKER,
      SNAKING_RAINBOW,
      HIGHLIGHT,
    }
    //TODO: Set the desired color for the Elevator target
    private static int[] ELEVATOR_TARGET_COLOR = {0,0,0};
  }

  private static AddressableLED led;//Led Strip
  private static AddressableLEDBuffer ledBuffer;//Buffer used before displaying to the strip
  private int[] patternColor = {255,255,255};//Pattern color for use in the Highlight and Random
  private int[] backgroundColor = {0,0,0};//Background color used in the Random
  private int previousColorLimit;//Store the color limit for rainbow. When the rainbow is called during periodic, this value is used to store the 
  //previous position that the strip was at.
  private int previousSlowFillLED;//Stores the LED that it was last used.
  private int previousSnakingLED;//Stores the LED that was last at. Useful in making sure there are no loop overruns
  private double elevatorLowerPos;//Lower Elevator position as a percent of the strip
  private double elevatorUpperPos;//Maximum Elevator position for a given scoring position as a percent
  private Distance setElevatorPosition;//Set point of the elevator
  private static int TRAILING_BRIGHTNESS = 10;//How long the brightness chain lasts. Larger is a smaller trail, smaller is a larger trail
  private static int RIO_PIN = 0;//TODO: Determine which pin of the Rio we are going to use the LEDs on
  private int delayRobotCycles = 0;//The amount of robot cycles to wait for
  private int currentRobotCycle;//Current amount of waited robot cycles

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    //Setting the pin of the Rio that the LED strip is on
    led = new AddressableLED(RIO_PIN);
    //Creating the buffer that is used for setting the data
    ledBuffer = new AddressableLEDBuffer(LEDConstants.NUMBER_OF_LEDS);
    //Setting the length of the LED strip
    led.setLength(LEDConstants.NUMBER_OF_LEDS);
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
  public void setLED(frc.robot.LEDSubsystem.LEDConstants.LED_MODES mode){
    switch(mode){
      case RAINBOW: 
        //"Overflow" that sets the color limit to 0
        if(previousColorLimit > 255) previousColorLimit = 0;
        else previousColorLimit++;//Increment the color
        //Loops over the LEDs in the strip
        for(int LEDNum = 0; LEDNum < LEDConstants.NUMBER_OF_LEDS; LEDNum++){
          //Creates a new color that gets from the rainbowCoor function, taking in the 
          //color limit and the current LED 
          int color[] = rainbowColor(previousColorLimit + LEDNum);
          //Update buffer
          ledBuffer.setRGB(LEDNum, color[0], color[1], color[2]);
        }
        //Display the color
        led.setData(ledBuffer);
      break;

      case SLOW_FILL: 
        if(previousSlowFillLED > LEDConstants.NUMBER_OF_LEDS) previousSlowFillLED = 0; 
        else previousSlowFillLED++;
        //Sets the current LED to the pattern color
        ledBuffer.setRGB(previousSlowFillLED, patternColor[0], patternColor[1], patternColor[2]);
        //Updates the LED strand
        led.setData(ledBuffer);
      break;

      case RANDOM_FLICKER: 
        //Random Object
        Random random = new Random();
        //Generating the Random LEDs to change
        int[] ledPosition = {
          random.nextInt(LEDConstants.NUMBER_OF_LEDS),
          random.nextInt(LEDConstants.NUMBER_OF_LEDS),
          random.nextInt(LEDConstants.NUMBER_OF_LEDS),
        };
        //Clear the background
        LEDPattern.solid(new Color(backgroundColor[0],backgroundColor[1],backgroundColor[2])).applyTo(ledBuffer);
        //Updates the strip to the current ones
        ledBuffer.setRGB(ledPosition[0], patternColor[0], patternColor[1], patternColor[2]);
        ledBuffer.setRGB(ledPosition[1], patternColor[0], patternColor[1], patternColor[2]);
        ledBuffer.setRGB(ledPosition[2], patternColor[0], patternColor[1], patternColor[2]);
        //Update the string
        led.setData(ledBuffer);
      break;

      case SNAKING_RAINBOW: 
        if(previousSnakingLED > 255) previousSnakingLED = 0;
        else previousSnakingLED++;
        //Loops over the LEDs in the strip
        for(int LEDNum = 0; LEDNum < LEDConstants.NUMBER_OF_LEDS; LEDNum++){
          //Creates a new color that gets from the rainbowCoor function, taking in the 
          //color limit and the current LED 
          int color[] = rainbowColor(previousColorLimit + LEDNum);
          //Adding the tail by decreasing the "brightness" by 10 for each LED
          color[0] -= previousSnakingLED * TRAILING_BRIGHTNESS;
          color[1] -= previousSnakingLED * TRAILING_BRIGHTNESS;
          color[2] -= previousSnakingLED * TRAILING_BRIGHTNESS;
          //Verify that it is not negative. If so, make it 0
          if(color[0] < 0) color[0] = 0;
          if(color[1] < 0) color[1] = 0;
          if(color[2] < 0) color[2] = 0;
          //Update buffer
          ledBuffer.setRGB(LEDNum, color[0], color[1], color[2]);
        }
        led.setData(ledBuffer);
      break;

      case FLICKER:
        if(currentRobotCycle == delayRobotCycles){
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
        led.setData(ledBuffer);//Update the strip
      break;
    }
  }

  /**
   * @breif   Sets the color of the pattern
   * @param color   The color to set to
   */
  public void setPatternColor(int color[]){ patternColor = color; }

  /**
   * @breif   Sets the color of the background when using the random flicker
   * @param color   The color to set to
   */
  public void setBackgroundColor(int color[]){ backgroundColor = color; }

  /**
   * @breif   Takes in the current position of the Elevator and uses it to display where the elevator will be going
   * @param position  The Distance object of the Elevator position
   * @param currentElevatorHeight  The current height of the elevator, given in meters
   */
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
  }

  /**
   * @breif   Turns the LEDs off
   */
  public void off(){
    LEDPattern.solid(Color.kBlack).applyTo(ledBuffer);
    led.setData(ledBuffer);
  }


  /**
   * @breif  Sets the delay for the Random Flicker and Flicker Effects in number of Robot cycles. For instance, a 
   * value of 1 will mean that every other robot cycle, it will flicker
   * @param delay   The number of Robot Periodic Cycles to wait for
   */
  public void setPeriodicDelay(int delay){
    delayRobotCycles = delay;
  }

  /**
   * @breif  Gets the current delay of Robot Periodic Cycles
   * @return  The number of Cycles
   */
  public int getPeriodicDelay(){
    return delayRobotCycles;
  }

  //Used for determine the color to be assigned to an LED based on its position on the strip
  //and the current color limit
  private int[] rainbowColor(int wheelPos){
    int r, g, b;
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
    else {
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
    //Nothing in here, for now...
  }
}


