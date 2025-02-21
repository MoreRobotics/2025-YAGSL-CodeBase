// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  public double runFunnelSpeed = -0.5;


  private SparkMax m_Funnel;
  private SparkMaxConfig funnelConfig;
  /** Creates a new Funnel. */
  public Funnel() {
    m_Funnel =  new SparkMax(m_FunnelID, MotorType.kBrushless);
    funnelConfig = new SparkMaxConfig();

    // funnelConfig.encoder
    //   .positionConversionFactor(funnelGearRatio)
    //   .velocityConversionFactor(funnelGearRatio);
    funnelConfig
    .inverted(false)
    .idleMode(IdleMode.kCoast);
  }

  public void runFunnel(double speed) {
    m_Funnel.set(speed);
  }

  public void stopFunnel() {
    m_Funnel.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
