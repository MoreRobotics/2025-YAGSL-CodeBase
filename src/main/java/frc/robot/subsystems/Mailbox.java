// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Mailbox extends SubsystemBase {
  private int MailboxID = 12;
  private int rMailboxID = 11;
  private int sensorID = 0;
  private double mailboxCurrentLimit = 20;
  private double mailboxP = 1.0;
  private double mailboxI = 0.0;
  private double mailboxD = 0.0;

  // TODO: Update the gear ratios
  private double lMailboxGearRatio = 1.0;
  private double rMailboxGearRatio = 1.0;

  public double intakeSpeed = 0.25;
  public double outtakeSpeed = 1.0;

  private TalonFX m_Mailbox;
  private DigitalInput sensor;
  private CurrentLimitsConfigs currentLimitConfig;
  private MotorOutputConfigs outputConfigs;
  private Slot0Configs pid;
  private VelocityVoltage m_request;

  /** Creates a new Mailbox. */
  public Mailbox() {
    m_Mailbox = new TalonFX(MailboxID);
    sensor = new DigitalInput(sensorID);
    m_request = new VelocityVoltage(0.0).withSlot(0);


    pid = new Slot0Configs();
    pid.kP = mailboxP;
    pid.kI = mailboxI;
    pid.kD = mailboxD;

    currentLimitConfig = new CurrentLimitsConfigs()
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(mailboxCurrentLimit);
    outputConfigs = new MotorOutputConfigs()
      .withInverted(InvertedValue.Clockwise_Positive)
      .withNeutralMode(NeutralModeValue.Coast);

    m_Mailbox.getConfigurator().apply(outputConfigs);
    m_Mailbox.getConfigurator().apply(pid);
    m_Mailbox.getConfigurator().apply(currentLimitConfig);
    
  }

  public void setMailboxSpeed(double speed) {
    m_Mailbox.setControl(m_request.withVelocity(speed));
  }

  public boolean getSensorInput() {
    return sensor.get();
  }

  public Trigger getSensorTrigger() {
    return new Trigger(() -> sensor.get());
  }

  public void stopMailBox() {
    m_Mailbox.setControl(m_request.withVelocity(0.0));
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Corral Sensor", getSensorInput());
  }
}
