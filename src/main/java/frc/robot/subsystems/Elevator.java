// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import java.sql.DriverPropertyInfo;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DifferentialPositionVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;


public class Elevator extends SubsystemBase {

  private final int m_ElevatorID = 2;
  private final int e_ElevatorID = 2;

  private int elevatorCurrentLimit = 60;

  public double targetElevatorPosition = 0;

  private double heightlimit = 16;
  public double elevatorspeed = 0.1;
  public double restingposition = 0;

  private TalonFX m_Elevator;
  private CANcoder e_Elevator;

  private final double m_ElevatorPGains = 0.0;
  private final double m_ElevatorIGains = 0.0;
  private final double m_ElevatorDGains = 0.0;
  private final double m_ElevatorFF = 5.0;
  private final double magnetOffset = 0.0;

  private Slot0Configs slotConfigs;
  private FeedbackConfigs feedbackConfigs;
  private CurrentLimitsConfigs currentLimitConfigs;
  private SoftwareLimitSwitchConfigs softLimitConfigs;
  private MagnetSensorConfigs magnetConfigs;

  private TalonFXConfigurator config;
  private PositionVoltage m_Request;
  private double voltage = 0.0;


  /** Creates a new Elevator. */
  public Elevator() {
    m_Elevator = new TalonFX(m_ElevatorID);
    e_Elevator = new CANcoder(e_ElevatorID);

    m_Elevator.getConfigurator().apply(slotConfigs);
    m_Elevator.getConfigurator().apply(feedbackConfigs);
    m_Elevator.getConfigurator().apply(softLimitConfigs);
    m_Elevator.getConfigurator().apply(currentLimitConfigs);

    e_Elevator.getConfigurator().apply(magnetConfigs);


    currentLimitConfigs = new CurrentLimitsConfigs()
    .withSupplyCurrentLimitEnable(true)
    .withSupplyCurrentLimit(elevatorCurrentLimit);

    feedbackConfigs = new FeedbackConfigs()
    .withSensorToMechanismRatio(Constants.ELEVATOR_ROTATIONS_TO_IN);

    softLimitConfigs = new SoftwareLimitSwitchConfigs()
    .withForwardSoftLimitEnable(true)
    .withForwardSoftLimitThreshold(inchesToMotorRotations(heightlimit));

    magnetConfigs = new MagnetSensorConfigs()
    .withMagnetOffset(magnetOffset);
    

    slotConfigs = new Slot0Configs();
    slotConfigs.kP = m_ElevatorPGains;
    slotConfigs.kI = m_ElevatorIGains;
    slotConfigs.kD = m_ElevatorDGains;


    Timer.delay(1.0);
    setElevatorEncoder();
    

  }

  public void setElevatorPosition(double inches) {
          m_Elevator.setControl(m_Request.withPosition(inches).withFeedForward(m_ElevatorFF));
  }

public void endElevator() {
  m_Elevator.setVoltage(0);
}

  private double inchesToMotorRotations(double inches) {
    return inches / Constants.ELEVATOR_ROTATIONS_TO_IN;
  }

  public double getPosition() {
    return m_Elevator.getPosition().getValueAsDouble();
  }

  public void stop() {
    m_Elevator.setPosition(0.0);
  }

  public void setElevatorEncoder() {
    m_Elevator.setPosition(e_Elevator.getPosition().getValue());
  }


  public boolean atPosition() {
    double error = Math.abs(getPosition() - targetElevatorPosition);

    if (Constants.ELEVATOR_TOLERANCE >= error) {
      return true;
    } else {
      return false;
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
