// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private int m_CimberID = 13;
  private int e_ClimberID = 15;
  private double climberP = 1.0;
  private double climberI = 0.0;
  private double climberD = 0.0;
  private double magnetOffset = 0.0;
  private double climberGearRatio = 625;
  private double currentLimit = 0.0;
  private double climberFF = 0.0; 

  private Slot0Configs slotConfigs;
  private TalonFX m_Climber;
  private CANcoder e_Climber;
  private CurrentLimitsConfigs currentLimitConfigs;
  private FeedbackConfigs feedbackConfigs;
  private ArmFeedforward feedforward;
  private PositionVoltage m_Request;
  /** Creates a new Climber. */
  public Climber() {

    m_Climber = new TalonFX(m_CimberID);
    e_Climber = new CANcoder(e_ClimberID);
    m_Request = new PositionVoltage(0).withSlot(0);

    currentLimitConfigs = new CurrentLimitsConfigs();
    currentLimitConfigs.SupplyCurrentLimitEnable = true;
    currentLimitConfigs.SupplyCurrentLimit = currentLimit;
    feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.SensorToMechanismRatio = climberGearRatio;
    slotConfigs = new Slot0Configs();
    slotConfigs.kP = climberP;
    slotConfigs.kI = climberI;
    slotConfigs.kD = climberD;

    m_Climber.getConfigurator().apply(slotConfigs);
    m_Climber.getConfigurator().apply(feedbackConfigs);
    m_Climber.getConfigurator().apply(currentLimitConfigs);
    m_Climber.setNeutralMode(NeutralModeValue.Brake);

    CANcoderConfigurator cancoderConfigurator = e_Climber.getConfigurator();

    cancoderConfigurator.apply(
      new MagnetSensorConfigs()
      .withMagnetOffset(magnetOffset)
    );

    setInternalEncoder();

 

  }

  public void setClimberPosition(double setpoint) {
    m_Climber.setControl(m_Request.withPosition(setpoint));
  }

  public void setInternalEncoder() {
    m_Climber.setPosition(e_Climber.getPosition().getValueAsDouble());
  }

  public double cancoderInDegrees() {
    return e_Climber.getPosition().getValueAsDouble() * 360;
  }

  public void stopClimber() {
    m_Climber.setVoltage(2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Position", m_Climber.getPosition().getValueAsDouble());
  }
}
