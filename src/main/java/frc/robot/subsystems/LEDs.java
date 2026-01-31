// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
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


  /** Creates a new LEDs. */
  public LEDs() {
  }
  
  public Command setEffect(String name,Color newChosenColor1,Color newChosenColor2,Integer newChosenInteger,Double newChosenDouble) {
    TrobotAddressableLEDPattern chosen = null;
    if (name == "alternating") {chosen = alternating;}
    if (name == "blinking") {chosen = blinking;}
    if (name == "chaos") {chosen = chaos;}
    if (name == "chase") {chosen = chase;}
    if (name == "intensity") {chosen = intensity;}
    if (name == "rainbow") {chosen = rainbow;}
    if (name == "scanner") {chosen = scanner;}
    if (name == "solid") {chosen = solid;}
    final TrobotAddressableLEDPattern chosen2 = chosen;
    return new InstantCommand(
      () -> setEffectFunct(chosen2, newChosenColor1, newChosenColor2, newChosenInteger, newChosenDouble)
    );
  }

  public void setEffectFunct(TrobotAddressableLEDPattern effect,Color newChosenColor1,Color newChosenColor2,Integer newChosenInteger,Double newChosenDouble) {
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
