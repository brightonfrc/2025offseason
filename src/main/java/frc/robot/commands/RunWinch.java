// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Winch;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunWinch extends Command {
  private Winch winch;
  /** Creates a new RunWinch. */
  public RunWinch(Winch winch) {
    this.winch=winch;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(winch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    winch.runWinch();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    winch.stopWinch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
