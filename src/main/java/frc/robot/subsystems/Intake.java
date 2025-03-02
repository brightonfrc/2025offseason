// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LiftConstants;
import com.revrobotics.spark.SparkMax;

public class Intake extends SubsystemBase {
  /** Creates a new Arm. */
  private SparkMax intakeMotor;

  public Intake(SparkMax motor) {
    intakeMotor=motor;
  }
  /**Method used to set intake power to 60% in order to intake a coral or algae*/
  public void intake(){
    intakeMotor.set(IntakeConstants.intakePower);
  }
  /**Method used to set intake power to -20% to eject a coral or algae */
  public void outtake(){
    intakeMotor.set(IntakeConstants.outtakePower);
  }
  public void stop(){
    intakeMotor.set(0);
  }
}
