// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

import java.sql.DriverPropertyInfo;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DifferentialPositionVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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

  private int elevatorCurrentLimit = 65;
  private int elevatorCurrentLowerLimit = 30;


  public double targetElevatorPosition = 0;

  private final double heightlimit = 50;
  public double elevatorspeed = 0.1;
  public double restingposition = 5.8;//
  public double lvl1Position = 0.0;//
  public double lvl2Position = 5.81;//5.81
  public double lvl3Position = 21.59;//21.59
  public double lvl4Position = 46.31;//46.31
  public double algaeLvl2Position = 27.41;//27.41
  public double algaeLvl3Position = 41.17;//41.17
  public double algaeGroundPosition = 4.25;



  private TalonFX m_Elevator;
  private DigitalInput botSensor;

  private final double m_ElevatorPGains = 0.7;//0.85
  private final double m_ElevatorIGains = 0.05;//.1,
  private final double m_ElevatorDGains = 0.0;//0.15
  private final double m_ElevatorGGains = 0.45;//0.99
  // private final double m_ElevatorSGains = 0.8;//0.8
  // private final double m_ElevatorVGains = 0.001;
  private final double m_ElevatorAcceleration = 350.0;
  private final double m_ElevatorCoastV = 225.0;
  private final double magnetOffset = 0.0;
  private double target;
  private double tolerance = 2.0;

  private Slot0Configs slotConfigs;
  private FeedbackConfigs feedbackConfigs;
  private CurrentLimitsConfigs currentLimitConfigs;
  private SoftwareLimitSwitchConfigs softLimitConfigs;

  private TalonFXConfigurator config;
  private MotionMagicVoltage m_Request;
  private MotorOutputConfigs motorOutputConfigs;
  private MotionMagicConfigs motionMagicConfigs;
  private double voltage = 0.0;
  private boolean debounceSensor = true;


  /** Creates a new Elevator. */
  public Elevator() {
    m_Elevator = new TalonFX(m_ElevatorID);
    
     m_Request = new MotionMagicVoltage(0).withSlot(0);
    
    botSensor = new DigitalInput(m_BotSensorID);


    currentLimitConfigs = new CurrentLimitsConfigs()
    .withSupplyCurrentLimitEnable(true)
    .withSupplyCurrentLimit(elevatorCurrentLimit);
    // .withSupplyCurrentLowerLimit(elevatorCurrentLowerLimit);

    feedbackConfigs = new FeedbackConfigs()
    .withSensorToMechanismRatio(Constants.ELEVATOR_ROTATIONS_TO_IN);

    softLimitConfigs = new SoftwareLimitSwitchConfigs()
    .withForwardSoftLimitEnable(true)
    .withForwardSoftLimitThreshold(inchesToMotorRotations(heightlimit));

    motorOutputConfigs = new MotorOutputConfigs()
    .withInverted(InvertedValue.Clockwise_Positive)
    .withNeutralMode(NeutralModeValue.Brake);

    slotConfigs = new Slot0Configs().withGravityType(GravityTypeValue.Elevator_Static);
    slotConfigs.kP = m_ElevatorPGains;
    slotConfigs.kI = m_ElevatorIGains;
    slotConfigs.kD = m_ElevatorDGains;
    slotConfigs.kG = m_ElevatorGGains;
    // slotConfigs.kS = m_ElevatorSGains;
    //slotConfigs.kV = m_ElevatorVGains;

    motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicAcceleration = m_ElevatorAcceleration;
    motionMagicConfigs.MotionMagicCruiseVelocity = m_ElevatorCoastV;


    m_Elevator.getConfigurator().apply(motorOutputConfigs);
    m_Elevator.getConfigurator().apply(slotConfigs);
    m_Elevator.getConfigurator().apply(motionMagicConfigs);
    m_Elevator.getConfigurator().apply(feedbackConfigs);
    m_Elevator.getConfigurator().apply(currentLimitConfigs);

    
    Timer.delay(1.0);
    m_Elevator.setPosition(0);
    setElevatorPosition(lvl1Position);
  }

  public void setElevatorPosition(double inches) {

      m_Elevator.setControl(m_Request.withPosition(MathUtil.clamp(inches, 0, heightlimit)));
      target = inches;
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
    double currentPosition = m_Elevator.getPosition().getValueAsDouble();

    return(Math.abs(currentPosition - target)<tolerance);
  }

  public boolean getSensor() {
    return botSensor.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", m_Elevator.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Elevator Sensor", botSensor.get());
    SmartDashboard.putNumber("Elevator Target", target);
    SmartDashboard.putNumber("Elevator Current", m_Elevator.getSupplyCurrent().getValueAsDouble());


    if(botSensor.get() == false)
    {
      if (debounceSensor)
      {
        m_Elevator.setPosition(0.06);
        debounceSensor = false;
      }
    }
    else
    {
      debounceSensor = true;
    }
  }
}
