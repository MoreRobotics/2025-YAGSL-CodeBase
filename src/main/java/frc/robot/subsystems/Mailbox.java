// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
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
  private int lMailboxID = 12;
  private int rMailboxID = 11;
  private int sensorID = 0;
  private int mailboxCurrentLimit = 0;
  private double lMailboxP = 1.0;
  private double lMailboxI = 0.0;
  private double lMailboxD = 0.0;
  private double rMailboxP = 1.0;
  private double rMailboxI = 0.0;
  private double rMailboxD = 0.0;

  // TODO: Update the gear ratios
  private double lMailboxGearRatio = 1.0;
  private double rMailboxGearRatio = 1.0;

  public double runMailboxVoltage = -12.0;

  private SparkMax m_MailboxL;
  private SparkMax m_MailboxR;
  private DigitalInput sensor;
  private SparkMaxConfig lMailboxConfig;
  private SparkMaxConfig rMailboxConfig;

  /** Creates a new Mailbox. */
  public Mailbox() {
    m_MailboxL = new SparkMax(lMailboxID, MotorType.kBrushless);
    m_MailboxR = new SparkMax(rMailboxID, MotorType.kBrushless);
    sensor = new DigitalInput(sensorID);
    lMailboxConfig = new SparkMaxConfig();
    rMailboxConfig = new SparkMaxConfig();

    lMailboxConfig.encoder
      .positionConversionFactor(360 / lMailboxGearRatio)
      .velocityConversionFactor(360 / lMailboxGearRatio);
    lMailboxConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(lMailboxP, lMailboxI, lMailboxD);
    lMailboxConfig
      .inverted(true)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(mailboxCurrentLimit);


    rMailboxConfig.encoder
      .positionConversionFactor(360 / rMailboxGearRatio)
      .velocityConversionFactor(360 / rMailboxGearRatio);
    rMailboxConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(rMailboxP, rMailboxI, rMailboxD);
    rMailboxConfig
      .inverted(false)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(mailboxCurrentLimit);

    m_MailboxL.configure(lMailboxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_MailboxR.configure(rMailboxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setMailboxVolatge(double leftVoltage, double rightVoltage) {
    m_MailboxL.setVoltage(leftVoltage);
    m_MailboxR.setVoltage(rightVoltage);
  }

  public boolean getSensorInput() {
    return sensor.get();
  }

  public Trigger getSensorTrigger() {
    return new Trigger(() -> sensor.get());
  }

  public void stopMailBox() {
    m_MailboxL.setVoltage(0);
    m_MailboxR.setVoltage(0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Corral Sensor", getSensorInput());
  }
}
