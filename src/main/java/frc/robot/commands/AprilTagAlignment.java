package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AprilTagPoseEstimator;
import frc.robot.Constants.AprilTagAlignmentConstants;
import frc.robot.Constants.FieldOrientedDriveConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.Optional;

public class AprilTagAlignment extends Command {
  private final DriveSubsystem driveSubsystem;
  private final AprilTagPoseEstimator poseEstimator;
  private final PIDController movementXPID;
  private final PIDController movementYPID;


  private final PIDController rotationPID;

  public AprilTagAlignment(DriveSubsystem _driveSubsystem, AprilTagPoseEstimator _poseEstimator, double offsetX, double offsetY) {
    driveSubsystem = _driveSubsystem;
    poseEstimator = _poseEstimator;
    addRequirements(_driveSubsystem, _poseEstimator);

    movementXPID = new PIDController(
      AprilTagAlignmentConstants.kMoveP,
      AprilTagAlignmentConstants.kMoveI,
      AprilTagAlignmentConstants.kMoveD
    );

    movementYPID = new PIDController(
      AprilTagAlignmentConstants.kMoveP,
      AprilTagAlignmentConstants.kMoveI,
      AprilTagAlignmentConstants.kMoveD
    );

  rotationPID = new PIDController(
    AprilTagAlignmentConstants.kTurnP,
    AprilTagAlignmentConstants.kTurnI,
    AprilTagAlignmentConstants.kTurnD
  );

  rotationPID.setSetpoint(0);
  movementXPID.setSetpoint(offsetX);
  movementYPID.setSetpoint(offsetY);

    
    // Set tolerances for stopping
    movementXPID.setTolerance(AprilTagAlignmentConstants.errorIntervalPositions);
    movementYPID.setTolerance(AprilTagAlignmentConstants.errorIntervalPositions);
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

      double yaw = rotation.getZ();  // Rotation (Yaw)

      double rotationOutput = rotationPID.calculate(yaw);

      if(rotationPID.atSetpoint()){ // Move towards set point
        // Use PID  to calculate the movement speed needed to reduce error
        double movementSpeed = movementXPID.calculate(x, AprilTagAlignmentConstants.stopDistanceX); // Move x to 0
        double strafeSpeed = movementYPID.calculate(y, AprilTagAlignmentConstants.stopDistanceY);   // Move y to 0

        // Drive the robot towards the AprilTag
        driveSubsystem.drive(movementSpeed, -strafeSpeed, 0, false); // negative strafespeed because y is inversed (positive = left)
      }
      else{ // If not at set point, rotate towards setpoint
        driveSubsystem.drive(0, 0, rotationOutput, false);
      }
    }
    else endCommand = true;
  }

  private Boolean endCommand = false;

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false); // Stop the robot
  }

  @Override
  public boolean isFinished() {
    return (movementXPID.atSetpoint() && movementYPID.atSetpoint() && rotationPID.atSetpoint()) || endCommand; // Stops when within error tolerance
  }
}
