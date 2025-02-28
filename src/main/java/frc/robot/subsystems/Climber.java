// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private int m_CimberID = 13;
  private int e_ClimberID = 15;
  private double climberP = 200.0; //24
  private double climberI = 0.0;
  private double climberD = 0.0;
  private double climberLoadedP = 3.0;
  private double climberLoadedI = 0.1; //3
  private double climberLoadedD = 0.1;

  private double magnetOffset = 0.0;
  private double climberGearRatio = 300;
  private double currentLimit = 60.0;
  private double climberFF = 0.0; 
  private double target = 0.15;
  private double forwardLimit = 0.31;
  private double reverseLimit = -0.37;
  
  private double climberSafePose = -0.05;//between ready and end climb
  private double climberReadyPose = 0.20;
  private double climberEndPose = -0.19; //0.27
  public boolean hasClimbed = false;
  private double tolerance = 0.2;

  private Slot0Configs pidConfig;
  private Slot1Configs loadedPidConfig;
  private TalonFX m_Climber;
  private CANcoder e_Climber;
  private CurrentLimitsConfigs currentLimitConfig;
  private FeedbackConfigs feedbackConfig;
  private ArmFeedforward feedforward;
  private PositionVoltage m_Request;
  private MotorOutputConfigs motorOutputConfigs;
  private SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs;
  private ClosedLoopGeneralConfigs closedLoopGeneralConfigs;
  /** Creates a new Climber. */
  public Climber() {

    m_Climber = new TalonFX(m_CimberID);
    e_Climber = new CANcoder(e_ClimberID);
    m_Request = new PositionVoltage(0).withSlot(0);
 
    pidConfig = new Slot0Configs();
      pidConfig.kP = climberP;
      pidConfig.kI = climberI;
      pidConfig.kD = climberD;

    loadedPidConfig = new Slot1Configs();
      loadedPidConfig.kP = climberLoadedP;
      loadedPidConfig.kI = climberLoadedI;
      loadedPidConfig.kD = climberLoadedD;

    closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs()
      .withContinuousWrap(false);

    softwareLimitSwitchConfigs =  new SoftwareLimitSwitchConfigs()
      .withForwardSoftLimitEnable(true)
      .withForwardSoftLimitThreshold(forwardLimit)
      .withReverseSoftLimitEnable(true)
      .withReverseSoftLimitThreshold(reverseLimit);

    feedbackConfig = new FeedbackConfigs()
      .withSensorToMechanismRatio(climberGearRatio);
      
    currentLimitConfig = new CurrentLimitsConfigs()
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(currentLimit);

    motorOutputConfigs = new MotorOutputConfigs()
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withNeutralMode(NeutralModeValue.Brake);

    m_Climber.getConfigurator().apply(motorOutputConfigs);
    m_Climber.getConfigurator().apply(pidConfig);
    m_Climber.getConfigurator().apply(loadedPidConfig);
    m_Climber.getConfigurator().apply(feedbackConfig);
    m_Climber.getConfigurator().apply(currentLimitConfig);
    m_Climber.getConfigurator().apply(softwareLimitSwitchConfigs);
    m_Climber.getConfigurator().apply(closedLoopGeneralConfigs);

    e_Climber.getConfigurator().apply(
      new MagnetSensorConfigs()
      .withMagnetOffset(magnetOffset)
    );

    Timer.delay(1.0);
    setInternalEncoder();
    setClimberSafePose();

  }

  public void changeTarget() {
    if (hasClimbed == false) {
      target = climberReadyPose;
    } else {
      target = climberEndPose;
    }
  }

  public boolean checkClimb() {
    return hasClimbed = !hasClimbed;
  }

  public void setClimberPosition() {
    m_Climber.setControl(m_Request.withPosition(target));
  }

  public void setClimberSafePose() {
    target = climberSafePose;
    m_Climber.setControl(m_Request.withPosition(target));
  }

  public boolean atPosition() {
    double currentPosition = m_Climber.getPosition().getValueAsDouble();
    System.out.println(Math.abs(currentPosition - target));

    return(Math.abs(currentPosition - target)<tolerance);
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
    SmartDashboard.putNumber("Climber CANCoder", e_Climber.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Climber Set Point", target);
    SmartDashboard.putNumber("Climber Current", m_Climber.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Climber Voltage", m_Climber.getSupplyVoltage().getValueAsDouble());
    SmartDashboard.putBoolean("Toggle Value", hasClimbed);
  }
}
