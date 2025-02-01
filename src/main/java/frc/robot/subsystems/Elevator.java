// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private final int m1ElevatorID = 2;
  private final int m2ElevatorID = 3;

  private int elevatorCurrentLimit = 60;

  public double targetElevatorPosition = 0;

  private double heightlimit = 16;
  public double elevatorspeed = 0.1;
  public double restingposition = 0;

  private TalonFX m1Elevator;
  private TalonFX m2Elevator;

  private final double m_ElevatorPGains = 0.0;
  private final double m_ElevatorIGains = 0.0;
  private final double m_ElevatorDGains = 0.0;

  private Slot0Configs slotConfigs1;
  private Slot0Configs slotConfigs2;

  private TalonFXConfigurator config;
  private FeedbackConfigs eConfigs;

  private double voltage = 0;


  /** Creates a new Elevator. */
  public Elevator() {
    m1Elevator = new TalonFX(m1ElevatorID);
    m2Elevator =new TalonFX(m2ElevatorID);

    config = m1Elevator.getConfigurator();

    config.apply(
      new CurrentLimitsConfigs()
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(elevatorCurrentLimit)
    );

    config.apply(
      new FeedbackConfigs()
      .withSensorToMechanismRatio(Constants.ELEVATOR_ROTATIONS_TO_IN)
    );

    slotConfigs1 = new Slot0Configs();
    slotConfigs1.kP = m_ElevatorPGains;
    slotConfigs1.kI = m_ElevatorIGains;
    slotConfigs1.kD = m_ElevatorDGains;

    slotConfigs2 = new Slot0Configs();
    slotConfigs2.kP = m_ElevatorPGains;
    slotConfigs2.kI = m_ElevatorIGains;
    slotConfigs2.kD = m_ElevatorDGains;

    

    elevatorMotorConfig();
  }

  public void setElevatorVoltage(double inches) {
          targetElevatorPosition = inches;
  }

  public void elevatorMotorConfig() {
    m1Elevator.getConfigurator().apply(slotConfigs1);
    m2Elevator.getConfigurator().apply(slotConfigs2);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
