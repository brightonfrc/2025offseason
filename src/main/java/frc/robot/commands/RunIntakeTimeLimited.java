// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.LiftConstants.Height;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DatisLift;
import frc.robot.subsystems.Intake;

public class RunIntakeTimeLimited extends Command {
  private Intake intake;
  private Boolean in;
  private DatisLift lift;
  private Arm arm;
  private Boolean l4Outake;
  private PIDController liftController;
  private PIDController armController;
  private long startTimeMillis = 0; // Store the start time manually

  private double setTime = 1;

  public RunIntakeTimeLimited(Intake intake, Boolean in, DatisLift lift, Arm arm, double setTime) {
    this.intake = intake;
    this.in = in;
    this.lift = lift;
    this.setTime = setTime;

    addRequirements(intake, lift, arm);

  }

  @Override
  public void initialize() {
    startTimeMillis = System.currentTimeMillis(); // Record the start time

    if (!in) {
      l4Outake = true;
      liftController = new PIDController(LiftConstants.kPLift, LiftConstants.kILift, LiftConstants.kDLift);
      liftController.setTolerance(LiftConstants.angleTolerance);
      liftController.setSetpoint(LiftConstants.L4OuttakeEnd);
    } else {
      l4Outake = false;
    }
  }

  @Override
  public void execute() {
    if (l4Outake) {  
      double currentAngle = Math.toRadians(lift.getLiftAngle());
      double desiredPower = liftController.calculate(currentAngle);
      desiredPower += LiftConstants.kWeightMomentOffsetFactor * Math.cos(currentAngle);
      lift.setPower(desiredPower);

      if (Math.toRadians(currentAngle) < LiftConstants.L4OuttakeAngle) {
        intake.outtake();
      }
    } else {
      if (in) {
        intake.intake();
      } else {
        intake.outtake();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    lift.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - startTimeMillis) >= setTime * 1000; // Stop after x seconds
  }
}
