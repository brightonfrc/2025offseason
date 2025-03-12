// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LiftConstants;
import com.revrobotics.spark.SparkMax;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private DutyCycleEncoder encoder;
  private SparkMax armMotor;
  private double previousPowerSet;
  
  public Arm(DutyCycleEncoder encoder, SparkMax motor) {
    this.encoder=encoder;
    armMotor=motor;
    previousPowerSet=0;
  }
  /**Gets the arm angle above or below the horizontal formed by the lift. So being below would create a negative value if arm below lift
   * @returns arm angle in degrees
  */
  public double getArmAngle(){
    double measuredAngle= encoder.get();
    measuredAngle=measuredAngle*360;
    //in case you want to round the value
    // measuredAngle = Math.round(measuredAngle * Math.pow(10, ArmConstants.decimalPlaces))/ Math.pow(10, ArmConstants.decimalPlaces);
    double angleAboveHorizontal=measuredAngle-ArmConstants.angleAtZero;
    //because its 360 when fully stowed downards
    if (angleAboveHorizontal>180){
      angleAboveHorizontal-=360;
    }
    SmartDashboard.putNumber("Raw Arm angle", measuredAngle);
    // SmartDashboard.putNumber("Arm angle", angleAboveHorizontal);
    return angleAboveHorizontal;
  }
  /**Method used to set arm power ranging from -1 to 1 */
  public void setPower(double power){
    //remember that setting positive power should make the arm go up relative to horizontal formed by the lift connected to it.
    power=-power;
    if(Math.abs(power)>ArmConstants.maxPower){
      if (power>0){
        power=LiftConstants.maxPower;
      }
      else{
        power=-LiftConstants.maxPower;
      }
    }
    armMotor.set(power);
    previousPowerSet=power;
  }
  public void lock(){
  }
  public double getPreviousPowerSet(){
    return previousPowerSet;
  }
}
