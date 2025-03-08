// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaePivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveAlgaePivot extends Command {
  AlgaePivot s_AlgaePivot;
  double target;
  /** Creates a new MoveAlgaePivot. */
  public MoveAlgaePivot(AlgaePivot s_AlgaePivot, double target) {
    this.s_AlgaePivot = s_AlgaePivot;
    this.target = target;
    addRequirements(s_AlgaePivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_AlgaePivot.moveAlgaePivot(target);
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
    return s_AlgaePivot.atPosition();
  }
}
