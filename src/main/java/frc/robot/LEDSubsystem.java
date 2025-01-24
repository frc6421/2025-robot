// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  public static class LEDConstants {

    public static int NUMBER_OF_LEDS = 100;
    public static int OFF[] = {0,0,0}, 
    WHITE[] = {255,255,255};
    public static enum LED_MODES{
      RAINBOW,
      SLOW_FILL,
      RANDOM_FLICKER,
      SNAKING_RAINBOW
    }

    public static enum LEDColors{
      OFF,
      HOT_PINK,
      BLUE,
      PURPLE,
      YELLOW,
      GREEN,
      RED,
      ORANGE
    }

  }

  private static AddressableLED led;
  private static AddressableLEDBuffer ledBuffer;
  private int patternR = 255, patternG = 255, patternB = 255;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    led = new AddressableLED(0);
    ledBuffer = new AddressableLEDBuffer(LEDConstants.NUMBER_OF_LEDS);

    led.setLength(LEDConstants.NUMBER_OF_LEDS);
    led.setData(ledBuffer);
    led.start();

  }

  @Override
  public void periodic() {
  }
  //buffer.setRGB
  //buffer.getLength
  //led.setData(buffer)
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
  public void setLED(frc.robot.LEDSubsystem.LEDConstants.LED_MODES mode){
    switch(mode){
      case RAINBOW: 
        for(int colorLimit = 0; colorLimit <= 255; colorLimit++){
          for(int LEDNum = 0; LEDNum < ledBuffer.getLength(); LEDNum++){
            int color[] = rainbowColor(colorLimit + LEDNum);
            ledBuffer.setRGB(LEDNum, color[0], color[1], color[2]);
          }
          led.setData(ledBuffer);
        }
      break;
      case SLOW_FILL: 
        for(int LEDNum = 0; LEDNum < ledBuffer.getLength(); LEDNum++){
          //Sets the current LED to the pattern color
          ledBuffer.setRGB(LEDNum, patternR, patternG, patternB);
          //Updates the LED strand
          led.setData(ledBuffer);
        }
      break;
      case RANDOM_FLICKER: break;
      case SNAKING_RAINBOW: break;
    }
  }
  /**
   * @breif   Sets the color of the pattern
   * @param color   The color to set to
   */
  public void setPatternColor(int color[]){}

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
    //Why I had to do this, don't know
    //TODO: Find out a way to return an array without having to make a whole new variable
    int[] values = {r,g,b};
    return values;
  }
}