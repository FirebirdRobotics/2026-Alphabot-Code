// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.trobot5013lib.led.AlternatingColorPattern;
import frc.robot.lib.trobot5013lib.led.BlinkingPattern;
import frc.robot.lib.trobot5013lib.led.ChaosPattern;
import frc.robot.lib.trobot5013lib.led.ChasePattern;
import frc.robot.lib.trobot5013lib.led.IntensityPattern;
import frc.robot.lib.trobot5013lib.led.RainbowPattern;
import frc.robot.lib.trobot5013lib.led.ScannerPattern;
import frc.robot.lib.trobot5013lib.led.SolidColorPattern;
import frc.robot.lib.trobot5013lib.led.TrobotAddressableLED;
import frc.robot.lib.trobot5013lib.led.TrobotAddressableLEDPattern;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDs extends SubsystemBase {
  int ledLength = 51;
  int ledPWMPort = 9;
  
  TrobotAddressableLED m_ledStrip = new TrobotAddressableLED(ledPWMPort, ledLength);
  
  
  Color white = new Color(253, 240, 213);
  Color red = new Color(139, 0, 0);
  Color blue = new Color(0, 0, 139);
  Color green = new Color(167, 201, 87);
  Color black = new Color(1,1,1); // Not sure if this will work
  //should be 1 for r value to get roughly black
  Color chosenColor = null;
  Color chosenColor2 = null;
  Integer chosenParameter = null;
  Double chosenParameter2 = null;

  Color[] chosenColors = {chosenColor, chosenColor2};

  AlternatingColorPattern alternating = new AlternatingColorPattern(chosenColors);
  BlinkingPattern blinking = new BlinkingPattern(chosenColor, chosenParameter);
  SolidColorPattern solid = new SolidColorPattern(chosenColor);
  ChaosPattern chaos = new ChaosPattern();
  ChasePattern chase = new ChasePattern(chosenColors, chosenParameter);
  IntensityPattern intensity = new IntensityPattern(chosenColor, chosenParameter);
  RainbowPattern rainbow = new RainbowPattern();
  ScannerPattern scanner = new ScannerPattern(chosenColor, chosenParameter);

  EndEffector m_EndEffector;

  String switchString = "black";


  public Command blinkBlackThenStayBlack(double blinkingTime) {
    return runEnd(
      () -> switchString = "black blinking",
      () -> switchString = "black blinking"
    ).withTimeout(blinkingTime); 
  }


  /** Creates a new LEDs. */
  public LEDs() {

  }
  
  // public void setChaos() {
  //   m_ledStrip.setPattern(chaos);
  // }

  // public void setBlinking(Color newChosenColor, double interval) {
  //   chosenColor = newChosenColor;
  //   chosenParameter2 = interval;
  //   m_ledStrip.setPattern(blinking);
  // }

  // public void setAlternating(Color newChosenColor1, Color newChosenColor2) {
  //   chosenColor = newChosenColor1;
  //   chosenColor2 = newChosenColor2;
  //   m_ledStrip.setPattern(alternating);
  // }

  // public void setChase(Color newChosenColor1, Color newChosenColor2, Integer parameter) {
  //   chosenColor = newChosenColor1;
  //   chosenColor2 = newChosenColor2;
  //   chosenParameter = parameter;
  //   m_ledStrip.setPattern(chase);
  // }

  // public void setIntensity(Color newChosenColor, Integer interval) {
  //   chosenColor = newChosenColor;
  //   chosenParameter = interval;
  //   m_ledStrip.setPattern(intensity);
  // }

  // public void setRainbow() {
  //   m_ledStrip.setPattern(rainbow);
  // }

  // public void setScanner(Color newChosenColor, Integer interval) {
  //   chosenColor = newChosenColor;
  //   chosenParameter = interval;
  //   m_ledStrip.setPattern(intensity);
  // }

  public void setEffect(TrobotAddressableLEDPattern effect,Color newChosenColor1,Color newChosenColor2,Integer newChosenInteger,Double newChosenDouble) {
    chosenColor = newChosenColor1;
    chosenColor2 = newChosenColor2;
    chosenParameter = newChosenInteger;
    chosenParameter2 = newChosenDouble;
    m_ledStrip.setPattern(effect);
  }


  @Override
  public void periodic() {
    
    
  }
}
