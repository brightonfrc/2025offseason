// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
public class DatisLift extends SubsystemBase {
  private SparkMax liftNeo;
  private SparkMax reversedLiftNeo;
  private AbsoluteEncoder encoder;
  /** Creates a new DatisLift. */
  public DatisLift(SparkMax liftNeo, SparkMax reversedLiftNeo) {
    this.liftNeo=liftNeo;
    this.reversedLiftNeo=reversedLiftNeo;
    encoder=liftNeo.getAbsoluteEncoder();
    
    
  }
  /**Method to get the current lift angle (0 being at lowest level) */
  public double getLiftAngle(){
    return encoder.getPosition()*LiftConstants.rotationConversionFactor;
  }
  /**Method used to set lift power ranging from -1 to 1 */
  public void setPower(double power){
    liftNeo.set(power);
    reversedLiftNeo.set(-power);
  }
  
}
