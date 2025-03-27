// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaePivot extends SubsystemBase {

  private CANcoder e_AlgaePivot;
  private SparkMax m_AlgaePivot;
  private SparkMaxConfig algaePivotConfig;
  private PIDController algaePivotPID;
  private CANcoderConfigurator cancoderConfigurator;
  private RelativeEncoder e_AlgaePivotIntegrated;

  private int e_AlgaePivotID = 1;
  private int m_AlgaePivotID = 10;
  private double algaePivotP = 0.07;//0.042
  private double algaePivotI = 0.0;
  private double algaePivotD = 0.0;
  private double m_setpoint = 52.1;

  //safe = -0.02, grab off reef = 0.19, lvl 3 = .28, ground = .28;
  public double safePose = 2.2;
  // public double stowPose = 52.1;
  public double reefLvl2 = 57.1;
  public double reefLvl3 = 57.1;
  public double groundPose = 61.61;
  public double processorPose = 20.2;
  private double algaePivotVoltage = 0;
  private double algaePivotGearRatio = 24.0;
  private double tolerance = 6.0;
  /** Creates a new AlgaePivot. */
  public AlgaePivot() {
    m_AlgaePivot = new SparkMax(m_AlgaePivotID, MotorType.kBrushless);
    e_AlgaePivot = new CANcoder(e_AlgaePivotID);
    algaePivotPID = new PIDController(algaePivotP, algaePivotI, algaePivotD);
    cancoderConfigurator =  e_AlgaePivot.getConfigurator();
    algaePivotConfig = new SparkMaxConfig();

    algaePivotConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    algaePivotConfig.encoder
      .positionConversionFactor(360/ algaePivotGearRatio);
    algaePivotConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    
    m_AlgaePivot.configure(algaePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_AlgaePivot.getEncoder().setPosition(cancoderInDegrees());
    moveAlgaePivot(safePose);

  }

  public void moveAlgaePivot(double setpoint) {
    m_setpoint = setpoint;
  }

  public double cancoderInDegrees() {
    return e_AlgaePivot.getPosition().getValueAsDouble() * 360;
  }

  public boolean atPosition() {
    double currentPosition = m_AlgaePivot.getEncoder().getPosition();
    return(Math.abs(currentPosition - m_setpoint)<tolerance);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    algaePivotVoltage = algaePivotPID.calculate(m_AlgaePivot.getEncoder().getPosition(), m_setpoint);
    m_AlgaePivot.setVoltage(algaePivotVoltage);

    SmartDashboard.putNumber("Algae Pivot Cancoder", cancoderInDegrees());
    SmartDashboard.putNumber("Algae Pivot Setpoint", m_setpoint);
    SmartDashboard.putNumber("Algae Pivot Internal Encoder", m_AlgaePivot.getEncoder().getPosition());
    SmartDashboard.putNumber("Algae Pivot Current", m_AlgaePivot.getOutputCurrent());
  }
}
