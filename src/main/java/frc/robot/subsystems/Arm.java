// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private AnalogEncoder encoder;
  private VictorSPX armMotor;

  public Arm(AnalogEncoder encoder, VictorSPX motor) {
    this.encoder=encoder;
    armMotor=motor;
  }
  /**Gets the arm angle above or below the horizontal. So being below would create a negative value */
  public double getArmAngle(){
    double measuredAngle= encoder.get();
    return measuredAngle;
  }
  /**Method used to set arm power ranging from -1 to 1 */
  public void setPower(double power){
    armMotor.set(VictorSPXControlMode.PercentOutput,power);
  }
}
