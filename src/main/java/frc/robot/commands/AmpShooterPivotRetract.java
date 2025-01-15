// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

public class AmpShooterPivotRetract extends Command {

  ShooterPivot s_ShooterPivot;
  
  /** Creates a new AmpScore. */
  public AmpShooterPivotRetract(ShooterPivot s_ShooterPivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_ShooterPivot = s_ShooterPivot;

    addRequirements(s_ShooterPivot);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotStowPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
