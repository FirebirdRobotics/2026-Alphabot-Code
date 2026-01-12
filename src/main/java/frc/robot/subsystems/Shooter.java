// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Shooter extends SubsystemBase {
  // Need to configure CAN ID's
  private final TalonFX m_leader = new TalonFX(ElevatorConstants.elevatorLeaderMotorID, "CANivore");
  private final TalonFX m_follower = new TalonFX(ElevatorConstants.elevatorFollowerMotorID, "CANivore");

  


  /** Creates a new Shooter. */
  public Shooter() {
    // Set follower motor to follow leader
    m_follower.setControl(new Follower(m_leader.getDeviceID(), MotorAlignmentValue.Aligned));



    var elevatorMotorConfigs = new TalonFXConfiguration();

    elevatorMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    elevatorMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    elevatorMotorConfigs.Feedback.SensorToMechanismRatio = ((19.65/7.75));

    elevatorMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorMotorConfigs.CurrentLimits.StatorCurrentLimit = 50;

    elevatorMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorMotorConfigs.CurrentLimits.SupplyCurrentLimit = 50;



    // set slot 0 gains
    var slot0Configs = elevatorMotorConfigs.Slot0;
    slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative    


    

    
    // set Motion Magic settings
    var motionMagicConfigs = elevatorMotorConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 100; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 30; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 0.0; // Target jerk of 1600 rps/s/s (0.1 seconds)

    m_leader.getConfigurator().apply(elevatorMotorConfigs);

  }

  // moves the elevator to intake height 
  public void goToHeight(double inches){

    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    m_leader.setControl(m_request.withPosition(inches));
  }

public Command testElevator() {

    return runOnce(() -> goToHeight(4.5));


    
  }

  public Command goToDereefHigh() {
    
    return runOnce(() -> goToHeight(ElevatorConstants.dereefHigh));

  }

  public Command goToDereefLow() {
    
    return runOnce(() -> goToHeight(ElevatorConstants.dereefLow));

  }

  

  public Command goToL4() {
    
    return runOnce(() -> goToHeight(ElevatorConstants.L4height));

  }

  public Command goToL3() {
    return runOnce(() -> goToHeight(ElevatorConstants.L3height));
    
  }

  public Command goToL2() {
    return runOnce(() -> goToHeight(ElevatorConstants.L2height));
    
  }

  public Command goToL1() {
    return runOnce(() -> goToHeight(ElevatorConstants.L1height));

  }

  public Command goToStowedPosition() {
    return runOnce(() -> goToHeight(ElevatorConstants.stowedPosition));
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // DogLog.log("Elevator position", m_leader.getPosition().getValueAsDouble());
    // DogLog.log("Elevator Velocity", m_leader.getVelocity().getValueAsDouble());
    // DogLog.log("Elevator Acceleration", m_leader.getAcceleration().getValueAsDouble());
    SmartDashboard.putNumber("Elevator position", m_leader.getPosition().getValueAsDouble());

  }

///////////////////////
public void visualizeElevator(double setpoint) {
  int height = (int) setpoint; // Convert setpoint to an integer for visualization
  int maxHeight = 100; // Maximum height for visualization (in inches)
  int scale = 2; // Scale factor for visualization (1 unit = 2 inches)

  System.out.println("Elevator Visualization:");
  for (int i = maxHeight; i >= 0; i -= scale) {
    if (i == height) {
      System.out.println("| [E] |"); // Elevator at the current height
    } else {
      System.out.println("|     |");
    }
  }
  System.out.println("-------");
  System.out.println("Resting Point: 0 inches");
}

////////////////////////////////////
private final ElevatorSim m_elevatorSim = new ElevatorSim(
    DCMotor.getKrakenX60(2), // Two Krakens
    10.0,                    // Gear ratio (motor rotations per drum rotation)
    5.0,                     // Carriage mass in kg (approximate for FRC)
    0.03,                    // Drum radius in meters (~1.2 in)
    0.0,                     // Min height (fully down)
    1.5,                     // Max height (approx 5 feet)
    true,                    // Simulate gravity
    0.01                     // Sensor noise
);

private final Mechanism2d m_mechanism2d = new Mechanism2d(2, 2); // 2x2 meter visualization
private final MechanismRoot2d m_elevatorRoot = m_mechanism2d.getRoot("Elevator Root", 1, 0);
private final MechanismLigament2d m_elevatorLigament = m_elevatorRoot.append(
  new MechanismLigament2d("Elevator", 0, 90)
);

@Override
public void simulationPeriodic() {
  // Update the elevator simulation
  m_elevatorSim.setInput(m_leader.getDutyCycle().getValue() * 12.0); // Convert percent output to volts
  m_elevatorSim.update(0.02); // Update simulation with a 20ms timestep

  // Update the Mechanism2d visualization
  double elevatorHeight = Units.metersToInches(m_elevatorSim.getPositionMeters());
  m_elevatorLigament.setLength(elevatorHeight / 50.0); // Scale height for visualization
  SmartDashboard.putData("Elevator Sim", m_mechanism2d);
}

}