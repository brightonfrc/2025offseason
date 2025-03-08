// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.LiftConstants.Height;
import frc.robot.subsystems.DatisLift;
import frc.robot.subsystems.Winch;
import frc.robot.commands.RunWinch;
import frc.robot.subsystems.Arm;
import frc.robot.commands.JankLift;;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HangRobot extends Command {
  private DatisLift lift;
  private PIDController liftController;
  // private Arm newarm;
  // private Winch win;
  /** Creates a new HangRobot. */

  public HangRobot(DatisLift lift) {
    this.lift=lift;
    // this.newarm = newarm;
    // this.win = win;
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // liftController = new PIDController(LiftConstants.kPHang, LiftConstants.kIHang, LiftConstants.kDHang);
    // liftController.setSetpoint(Math.toRadians(11));
    // liftController.setTolerance(LiftConstants.angleTolerance);
    // RunWinch runthing = new RunWinch(win);
    // runthing.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lift.setPower(liftController.calculate(Math.toRadians(lift.getLiftAngle())));
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
