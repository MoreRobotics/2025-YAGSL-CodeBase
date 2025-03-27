// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  private int m_AlgaeIntakeID = 16;
  private double algaeIntakeS = 0.1;
  private double algaeIntakeV = 0.12;
  private double algaeIntakeP = 0.3;
  private double algaeIntakeI = 0.0;
  private double algaeIntakeD = 0.0;
  public double algaeIntakeSpeed = 60.0;
  public double algaeOutakeSpeed = -40.0;
  public double algaeIdleSpeed = 25.0;
  private double gearRatio = 0.0;
  private double currentLimit = 80.0;
  private double m_Speed = 0.0;



  public TalonFX m_AlgaeIntake;
  private Slot0Configs pid;
  private MotorOutputConfigs outputConfigs;
  private VelocityVoltage m_Request;
  private CurrentLimitsConfigs currentLimitsConfigs;

  /** Creates a new Algae. */
  public AlgaeIntake() {
    m_AlgaeIntake = new TalonFX(m_AlgaeIntakeID);
    m_Request = new VelocityVoltage(0).withSlot(0);

    pid = new Slot0Configs();
    pid.kS = algaeIntakeS;
    pid.kV = algaeIntakeV;
    pid.kP = algaeIntakeP;
    pid.kI = algaeIntakeI;
    pid.kD = algaeIntakeD;

    outputConfigs = new MotorOutputConfigs()
      .withInverted(InvertedValue.Clockwise_Positive)
      .withNeutralMode(NeutralModeValue.Coast);
    currentLimitsConfigs = new CurrentLimitsConfigs()
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(currentLimit);

    m_AlgaeIntake.getConfigurator().apply(outputConfigs);
    m_AlgaeIntake.getConfigurator().apply(pid);
    m_AlgaeIntake.getConfigurator().apply(currentLimitsConfigs);

    runAlgaeIntake(algaeIdleSpeed);
  }

  public void runAlgaeIntake(double speed) {
    m_Speed = speed;
    m_AlgaeIntake.setControl(m_Request.withVelocity(speed));
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algae Intake Current", m_AlgaeIntake.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Algae Intake Set Speed", m_Speed);
    SmartDashboard.putNumber("Algae Intake Speed", m_AlgaeIntake.getVelocity().getValueAsDouble());
  }
}
