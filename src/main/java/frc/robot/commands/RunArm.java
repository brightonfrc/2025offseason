// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AngleLimitConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DatisLift;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunArm extends Command {
  private Boolean positive;
  private Arm arm;
  private DatisLift lift;
  private Boolean emergencyStop;
  /** Creates a new RunArm. */
  public RunArm(Arm arm, Boolean positive, DatisLift lift) {
    this.positive=positive;
    this.arm=arm;
    this.lift=lift;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // emergencyStop=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // SmartDashboard.putBoolean("command active", true);
    double liftAngle=lift.getLiftAngle();
    double armAngle=arm.getArmAngle();
    SmartDashboard.putNumber("Angle of arm", armAngle);
    SmartDashboard.putNumber("Angle above ground", liftAngle+armAngle);
    SmartDashboard.putBoolean("Command active", true);
    double weightOffsetFactor=ArmConstants.kWeightMomentOffsetFactor*Math.cos(Math.toRadians(armAngle+liftAngle));
    if (positive){
      // arm.setPower(weightOffsetFactor);
      arm.setPower(0.10);
    }
    else{
      // arm.setPower(weightOffsetFactor);
      arm.setPower(-0.10);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setPower(0);
    SmartDashboard.putBoolean("Command active", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return emergencyStop;
    return false;
  }
}
