// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.IntakeConstants;

public class FloorRollers extends SubsystemBase {
  /** Creates a new FloorRollers. */

  private final TalonFX m_floorroller = new TalonFX(41, "CANivore"); // change to different motor, idk what the motor is supposed to be

  public FloorRollers() {

    var rollerMotorConfigs = new TalonFXConfiguration();
  
    var intakeCANRangeConfigs = new CANrangeConfiguration();
  
    intakeCANRangeConfigs.ProximityParams.ProximityThreshold = 0.3;
  
    rollerMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    rollerMotorConfigs.Feedback.SensorToMechanismRatio = 23.265306122;
    rollerMotorConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    rollerMotorConfigs.CurrentLimits.StatorCurrentLimit = 60;
  
    rollerMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerMotorConfigs.CurrentLimits.StatorCurrentLimit = 50;
  
    rollerMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerMotorConfigs.CurrentLimits.SupplyCurrentLimit = 50;
  
  
    var motionMagicConfigs = rollerMotorConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 20.0; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 10.0; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 0.0; // Target jerk of 1600 rps/s/s (0.1 seconds)
  
    m_floorroller.getConfigurator().apply(rollerMotorConfigs);
  }

  boolean breaks = true;

  double rotations = m_floorroller.getPosition().getValueAsDouble();

  @Override
  public void periodic() {
    if (breaks == true) {
      final MotionMagicVoltage m_request = new MotionMagicVoltage(rotations);

      m_floorroller.setControl(m_request.withPosition(rotations));
    } else {
      rotations = m_floorroller.getPosition().getValueAsDouble();
    }
  }

  public void setRollerMotorPercentOutput(double outputPercent) {
    breaks = false;
    m_floorroller.setControl(new DutyCycleOut(outputPercent));
  }

  public Command StartTurn(double power) {
    return run(
      () -> setRollerMotorPercentOutput(power)
    );
  }
  public Command Break(double power) {
    return runEnd(
      () -> setRollerMotorPercentOutput(0),
      () -> breaks = true
    );
  }
}
