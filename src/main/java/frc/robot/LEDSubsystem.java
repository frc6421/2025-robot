// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  public static class LEDConstants {
    public static int NUMBER_OF_LEDS = 100;
                               // R   G   B
    public static int[] WHITE = {255,255,255},
                        RED   = {255, 0 , 0 },
                        GREEN = { 0 ,255, 0 },
                        BLUE  = { 0 , 0 ,255};
    public static enum LED_MODES{
      RAINBOW,
      SLOW_FILL,
      RANDOM_FLICKER,
      SNAKING_RAINBOW
    }

  }

  private static AddressableLED led;
  private static AddressableLEDBuffer ledBuffer;
  private int[] patternColor = {255,255,255};
  private int[] backgroundColor = {0,0,0};
  private int previousColorLimit;
  private int previousSlowFillLED;
  private int previousSnakingLED;
  private static int TRAILING_BRIGHTNESS = 10;//How long the brightness chain lasts. Larger is a smaller trail, smaller is a longer trail
  private static int RIO_PIN = 0;//TODO: Determine which pin of the Rio we are going to use the LEDs on

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
    for(int i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, color[0], color[1], color[2]);
    }
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
        for(int LEDNum = 0; LEDNum < ledBuffer.getLength(); LEDNum++){
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
        if(previousSlowFillLED > ledBuffer.getLength()) previousSlowFillLED = 0; 
        else previousSlowFillLED++;
        //Sets the current LED to the pattern color
        ledBuffer.setRGB(previousSlowFillLED, patternColor[0], patternColor[1], patternColor[2]);
        //Updates the LED strand
        led.setData(ledBuffer);
      break;

      case RANDOM_FLICKER: 
        Random random = new Random();
        int[] ledPosition = {
          random.nextInt(ledBuffer.getLength()),
          random.nextInt(ledBuffer.getLength()),
          random.nextInt(ledBuffer.getLength()),
        };
        for(int i = 0; i < ledBuffer.getLength(); i++){
          //Checks if the three postions are what i is currently at
          if(i == ledPosition[0] || i == ledPosition[1] || i == ledPosition[2]){
            //If so, sets to the pattern color
            ledBuffer.setRGB(i, patternColor[0], patternColor[1], patternColor[2]);
          }else{
            //If not, then set it to the background color. Also has the added benifit that it overwrites the previous pattern
            ledBuffer.setRGB(i, backgroundColor[0], backgroundColor[1], backgroundColor[2]);
          }
          led.setData(ledBuffer);
        }
      break;

      case SNAKING_RAINBOW: 
        if(previousSnakingLED > 255) previousSnakingLED = 0;
        else previousSnakingLED++;
        //Loops over the LEDs in the strip
        for(int LEDNum = 0; LEDNum < ledBuffer.getLength(); LEDNum++){
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
   * @breif   Turns the LEDs off
   */
  public void off(){
    for(int i = 0; i < ledBuffer.getLength(); i++) ledBuffer.setRGB(i,0,0,0);
    led.setData(ledBuffer);
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


