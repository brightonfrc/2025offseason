// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WinchConstants;

public class Winch extends SubsystemBase {
  private SparkMax winch;
  /** Creates a new Winch. */
  public Winch(SparkMax winchMotor) {
    winch=winchMotor;
  }
  /**Sets winch power to power specified in WinchConstants class */
  public void runWinch(){
    winch.set(WinchConstants.winchPower);
  }
  public void stopWinch(){
    winch.set(0);
  }
}
