// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {

  private final int m_AlgaeIntakeID = 2;
  private final int m_AlgaePivotID = 2;
  private final int e_AlgaePivotID = 2;
  private final double algaePivotP = 0.0;
  private final double algaePivotI = 0.0;
  private final double algaePivotD = 0.0;
  private final double algaePivotGearRatio = 0.0;
  private final double algaeIntakeGearRatio = 0.0;
  private final int algaePivotCurrentLimit = 0;
  private final int algaePivotCurrentLmit = 0;


  private SparkMax m_AlgaePivot;
  private SparkMax m_AlgaeIntake;
  private CANCoder e_AlgaePivot;
  private SparkMaxConfig algaePivotConfig;


  /** Creates a new AlgaePivot. */
  public AlgaeIntake() {
    m_AlgaePivot = new SparkMax(e_AlgaePivotID, MotorType.kBrushless);
    algaePivotConfig = new SparkMaxConfig();

    algaePivotConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(algaePivotP, algaePivotI, algaePivotD);

    algaePivotConfig.encoder
    .positionConversionFactor(360 / algaePivotGearRatio)
    .velocityConversionFactor(360 / algaePivotGearRatio);

    algaePivotConfig.smartCurrentLimit(algaePivotCurrentLimit);


    m_AlgaePivot.configure(algaePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
