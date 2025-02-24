// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {
  private int m_FunnelID = 10;
  private int currentLimit = 60;
  private double funnelGearRatio = 0.0;
  public double runFunnelSpeed = 1.0;
  private double funnelS = 0.1;
  private double funnelV = 0.12;
  private double funnelP = 0.11;
  private double funnelI = 0.0;
  private double funnelD = 0.0;


  private TalonFX m_Funnel;
  private Slot0Configs pid;
  private FeedbackConfigs feedbackConfigs;
  private MotorOutputConfigs outputConfigs;
  private VelocityVoltage m_request;

  /** Creates a new Funnel. */
  public Funnel() {
    m_Funnel =  new TalonFX(m_FunnelID);
    m_request = new VelocityVoltage(0.0).withSlot(0);
    
    
    pid = new Slot0Configs();
    pid.kP = funnelP;
    pid.kI = funnelI;
    pid.kD = funnelD;
    pid.kS = funnelS;
    pid.kV = funnelV;
    outputConfigs = new MotorOutputConfigs()
      .withInverted(InvertedValue.Clockwise_Positive)
      .withNeutralMode(NeutralModeValue.Coast);

  m_Funnel.getConfigurator().apply(outputConfigs);
  m_Funnel.getConfigurator().apply(pid);
 
    

    // funnelConfig.encoder
    //   .positionConversionFactor(funnelGearRatio)
    //   .velocityConversionFactor(funnelGearRatio);
    
  }

  public void runFunnel(double speed) {
    m_Funnel.setControl(m_request.withVelocity(speed));
  }

  public void stopFunnel() {
    m_Funnel.setControl(m_request.withVelocity(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
