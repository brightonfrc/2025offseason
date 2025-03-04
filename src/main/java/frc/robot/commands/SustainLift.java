// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DatisLift;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SustainLift extends Command {
  private double armPower;
  private double liftPower;
  private Arm arm;
  private DatisLift lift;
  /** Creates a new SustainLift. */
  public SustainLift(DatisLift lift, Arm arm) {
    this.arm=arm;
    this.lift=lift;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lift, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armPower=arm.getPreviousPowerSet();
    liftPower=lift.getPreviousPowerSet();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Command active", true);
    arm.setPower(armPower);
    lift.setPower(liftPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setPower(0);
    lift.setPower(0);
    SmartDashboard.putBoolean("Command active", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
