// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class AmpElevator extends Command {

  Elevator s_Elevator;
  
  /** Creates a new AmpScore. */
  public AmpElevator(Elevator s_Elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Elevator = s_Elevator;

    addRequirements(s_Elevator);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Elevator.SetElevatorPosition(Constants.ELEVATOR_HIGH_LEVEL);
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
