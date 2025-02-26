// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DatisLift;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.LiftConstants.Height;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Lift extends Command {
  private DatisLift lift;
  private double angleRequired;
  private PIDController liftController;
  private double previousPower;
  /** Creates a new Lift. */
  public Lift(DatisLift lift, Height heightDesired) {
    this.lift=lift;
    double height=0;
    switch(heightDesired){
      case Ground:
        height=LiftConstants.desiredHeight[0];
        break;
      case L1:
        height=LiftConstants.desiredHeight[1];
        break;
      case L2:
        height=LiftConstants.desiredHeight[2];
        break;
      case L3:
        height=LiftConstants.desiredHeight[3];
        break;
      case L4:
        height=LiftConstants.desiredHeight[4];
        break;
    }
    angleRequired=Math.asin((height/2)/LiftConstants.armLength);
    // SmartDashboard.putNumber("Angle required", Math.toDegrees(angleRequired));
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    liftController= new PIDController(LiftConstants.kPLift, LiftConstants.kILift, LiftConstants.kDLift);
    liftController.setTolerance(LiftConstants.angleTolerance);
    liftController.setSetpoint(angleRequired);
    previousPower=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Current angle", lift.getLiftAngle());
    double currentAngle=Math.toRadians(lift.getLiftAngle());
    double desiredPower=liftController.calculate(currentAngle);
    //scaling power down to reduce strain on lift
    desiredPower=desiredPower*LiftConstants.maxPower;
    if (Math.abs(desiredPower-previousPower)>LiftConstants.maximumPowerChange){
      //controller demands excessive power change
      if (desiredPower<0){
        //reducing power
        desiredPower=previousPower-LiftConstants.maximumPowerChange;
      }
      else{
        desiredPower=previousPower+LiftConstants.maximumPowerChange;
      }
    }
    desiredPower+=LiftConstants.kWeightMomentOffsetFactor*Math.cos(currentAngle);
    SmartDashboard.putNumber("Power", desiredPower);
    previousPower=desiredPower;
    lift.setPower(desiredPower);
    // SmartDashboard.putBoolean("Command active", !liftController.atSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lift.setPower(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return profiledLiftController.atSetpoint();
  }
}
