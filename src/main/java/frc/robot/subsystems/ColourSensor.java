// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColourSensorConstants;

public class ColourSensor extends SubsystemBase {
  private ColorSensorV3 sensor;
  private Color reefColor;
  private ColorMatch matcher;
  /** Creates a new ColourSensor. */
  public ColourSensor(ColorSensorV3 sensor) {
    reefColor=new Color(ColourSensorConstants.reefColorHex);
    this.sensor=sensor;
    matcher = new ColorMatch();
    matcher.addColorMatch(reefColor);
  }
  public void CheckReefAligned(){
    ColorMatchResult result = matcher.matchClosestColor(sensor.getColor());
    if (result.confidence>ColourSensorConstants.minimumConfidenceThreshold){
      SmartDashboard.putBoolean("Reef aligned", true);
    }
    else{
      SmartDashboard.putBoolean("Reef aligned", false);
    }
  }
}
