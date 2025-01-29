package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AprilTagPoseEstimator;
import frc.robot.Constants.AprilTagAlignmentConstants;

import java.util.Optional;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class AprilTagAlignment extends Command {
  private final DriveSubsystem driveSubsystem;
  private final AprilTagPoseEstimator poseEstimator;

  public AprilTagAlignment(DriveSubsystem _driveSubsystem, AprilTagPoseEstimator _poseEstimator) {
    driveSubsystem = _driveSubsystem;
    poseEstimator = _poseEstimator;
    addRequirements(_driveSubsystem, _poseEstimator);
  }

  double GetSign(double num){
    if (num == 0)
      return 0.0;
    else if (num > 0)
      return 1.0;
    else
      return -1.0;
  }

  @Override
  public void execute() {
    Optional<Transform3d> possibleTransform = poseEstimator.getRobotToTag(1); 

    if (possibleTransform.isPresent()) {
      Transform3d transform = possibleTransform.get();
      Translation3d translation = transform.getTranslation();
      Rotation3d rotation = transform.getRotation();

      double x = translation.getX(); // Forward/Backward
      double y = translation.getY(); // Left/Right
      double yaw = rotation.getZ();  // Rotation around the Z-axis (Yaw)

      // Constants for stopping thresholds and error intervals
      double tolerancePosition = AprilTagAlignmentConstants.errorIntervalPositions;
      double toleranceRotation = AprilTagAlignmentConstants.errorIntervalRotations;
      double xStopDistance = AprilTagAlignmentConstants.stopDistanceX;
      double yStopDistance = AprilTagAlignmentConstants.stopDistanceY;
      double speed = 0.3;
      double rotationSpeed = 0.2;

      // Movement calculations
      double forwardSpeed = Math.abs(x) > xStopDistance ? GetSign(x) * speed : 0;
      double sideSpeed = Math.abs(y) > yStopDistance ? GetSign(y) * speed : 0;
      double turnSpeed = Math.abs(yaw) > toleranceRotation ? GetSign(yaw) * rotationSpeed : 0;

      // Drive the robot
      driveSubsystem.drive(forwardSpeed, sideSpeed, turnSpeed, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false); // Stop movement when finished
  }

  @Override
  public boolean isFinished() {
    return false; // Adjust if needed
  }
}