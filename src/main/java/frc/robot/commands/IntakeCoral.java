// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Mailbox;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {

  Mailbox s_Mailbox;
  Funnel s_Funnel;
  /** Creates a new IntakeCoral. */
  public IntakeCoral(Mailbox s_Mailbox, Funnel s_Funnel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Mailbox = s_Mailbox;
    this.s_Funnel = s_Funnel;

    addRequirements(s_Mailbox, s_Funnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Funnel.runFunnel(s_Funnel.runFunnelVoltage);
    s_Mailbox.setMailboxVolatge(s_Mailbox.runMailboxVoltage,s_Mailbox.runMailboxVoltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Mailbox.stopMailBox();
    s_Funnel.stopFunnel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
