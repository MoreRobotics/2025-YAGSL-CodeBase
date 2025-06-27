// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Eyes;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignRight extends Command {
  Eyes s_Eyes;
  Swerve s_Swerve;
  /** Creates a new AutoAlignRight. */
  public AutoAlignRight(Eyes s_Eyes, Swerve s_Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Eyes = s_Eyes;
    this.s_Swerve = s_Swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Swerve.followPathCommand(() -> s_Eyes.closestRReefpath());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Swerve.followPathCommand(() -> s_Eyes.closestRReefpath());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
