// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Elevator extends SubsystemBase {

  private final int m_ElevatorID = 14;
  private final int m_BotSensorID = 1;

  private int elevatorCurrentLimit = 0;

  public double targetElevatorPosition = 0;

  private final double heightlimit = 197;
  public double elevatorspeed = 0.1;
  public double restingposition = 0;


  private TalonFX m_Elevator;
  private DigitalInput botSensor;

  private final double m_ElevatorPGains = 0.2;
  private final double m_ElevatorIGains = 1e-3;
  private final double m_ElevatorDGains = 0.0;
  private final double m_ElevatorFF = 0.0;
  private final double magnetOffset = 0.0;

  private Slot0Configs slotConfigs;
  private FeedbackConfigs feedbackConfigs;
  private CurrentLimitsConfigs currentLimitConfigs;
  private SoftwareLimitSwitchConfigs softLimitConfigs;

  private TalonFXConfigurator config;
  private PositionVoltage m_Request;
  private MotorOutputConfigs motorOutputConfigs;
  private double voltage = 0.0;


  /** Creates a new Elevator. */
  public Elevator() {
    m_Elevator = new TalonFX(m_ElevatorID);
    m_Request = new PositionVoltage(0).withSlot(0);
    
    botSensor = new DigitalInput(m_BotSensorID);


    // currentLimitConfigs = new CurrentLimitsConfigs()
    // .withSupplyCurrentLimitEnable(true)
    // .withSupplyCurrentLimit(elevatorCurrentLimit);

    feedbackConfigs = new FeedbackConfigs()
    .withSensorToMechanismRatio(Constants.ELEVATOR_ROTATIONS_TO_IN);

    softLimitConfigs = new SoftwareLimitSwitchConfigs()
    .withForwardSoftLimitEnable(true)
    .withForwardSoftLimitThreshold(inchesToMotorRotations(heightlimit));

    motorOutputConfigs = new MotorOutputConfigs()
    .withInverted(InvertedValue.Clockwise_Positive)
    .withNeutralMode(NeutralModeValue.Brake);

    slotConfigs = new Slot0Configs();
    slotConfigs.kP = m_ElevatorPGains;
    slotConfigs.kI = m_ElevatorIGains;
    slotConfigs.kD = m_ElevatorDGains;

    m_Elevator.getConfigurator().apply(motorOutputConfigs);
    m_Elevator.getConfigurator().apply(slotConfigs);
    m_Elevator.getConfigurator().apply(feedbackConfigs);

    
    Timer.delay(1.0);
    m_Elevator.setPosition(0);
    setElevatorPosition(restingposition);
  }

  public void setElevatorPosition(double inches) {

        m_Elevator.setControl(m_Request.withPosition(MathUtil.clamp(inches, 0, heightlimit)).withFeedForward(m_ElevatorFF));
      
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

  public void elevatorUp() {
    m_Elevator.setVoltage(3);
  }

  public void elevatorDown() {
    m_Elevator.setVoltage(-3);
  }



  public void stop() {
    m_Elevator.setPosition(0.0);
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
    SmartDashboard.putNumber("Elevator Position", m_Elevator.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Elevator Sensor", botSensor.get());

    // TODO uncomment below to enable encoder reset on sensor
    if(botSensor.get() == false)
    {
      m_Elevator.setPosition(0);
    }
  }
}
