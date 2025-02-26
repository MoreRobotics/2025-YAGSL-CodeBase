// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mailbox;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OutakeCoral extends Command {

  Mailbox s_Mailbox;
  /** Creates a new IntakeCoral. */
  public OutakeCoral(Mailbox s_Mailbox) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Mailbox = s_Mailbox;

    addRequirements(s_Mailbox);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
	  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Mailbox.setMailboxSpeed(s_Mailbox.outtakeSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Mailbox.stopMailBox();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
