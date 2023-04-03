// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  private AddressableLED led = new AddressableLED(4);
  private AddressableLEDBuffer ledData = new AddressableLEDBuffer(13);

  public static final int[] purple = {255,255,0};
  public static final int[] yellow = {255,255,0};
  public static final int[] red = {255,255,0};
  public static final int[] green = {255,255,0};
  public static final int[] white = {255,255,0};
  public static final int[] blue = {255,255,0};
  public static final int[] off = {0,0,0};


  /** Creates a new Lights. */
  public Lights() {
    led.setLength(ledData.getLength());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void showLights(String color) {
    if (color.equals("purple")) {
      for (int i = 0; i < ledData.getLength(); i++) {
        ledData.setRGB(i, purple[0], purple[1], purple[2]);
      }
    } else if (color.equals("yellow")) {
      for (int i = 0; i < ledData.getLength(); i++) {
        ledData.setRGB(i, yellow[0], yellow[1], yellow[2]);
      }
    
    } else if (color.equals("red")) {
      for (int i = 0; i < ledData.getLength(); i++) {
        ledData.setRGB(i, red[0], red[1], red[2]);
      }
    
    } else if (color.equals("green")) {
      for (int i = 0; i < ledData.getLength(); i++) {
        ledData.setRGB(i, green[0], green[1], green[2]);
      }
    
    } else if (color.equals("white")) {
      for (int i = 0; i < ledData.getLength(); i++) {
        ledData.setRGB(i, white[0], white[1], white[2]);
      }
    
    } else if (color.equals("blue")) {
      for (int i = 0; i < ledData.getLength(); i++) {
        ledData.setRGB(i, blue[0], blue[1], blue[2]);
      }
    } else {
      for (int i = 0; i < ledData.getLength(); i++) {
        ledData.setRGB(i, off[0], off[1], off[2]);
      }
    }
    led.setData(ledData);
    led.start();
  }
}
