// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AprilTagPoseEstimator extends SubsystemBase {
  private Optional<EstimatedRobotPose> prevEstimatedRobotPose;
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private final PhotonCamera cam;
  private final PhotonPoseEstimator photonPoseEstimator;

  /** Creates a new AprilTagPoseEstimator. */
  public AprilTagPoseEstimator() {
    // The field from AprilTagFields will be different depending on the game.
    this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    //Forward Camera
    this.cam = new PhotonCamera(Constants.CVConstants.kCameraName);

    // Construct PhotonPoseEstimator
    this.photonPoseEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, Constants.CVConstants.kRobotToCamera);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Optional<EstimatedRobotPose> getGlobalPose() {
    if(this.prevEstimatedRobotPose.isPresent()) {
      this.photonPoseEstimator.setReferencePose(this.prevEstimatedRobotPose.get().estimatedPose);
    }
    this.prevEstimatedRobotPose = photonPoseEstimator.update(cam.getLatestResult());
    return this.prevEstimatedRobotPose;
  }

  public Optional<Transform3d> getRobotToTag(int tagID) {
    // TODO: Use one tag only / is this even required? Right now will extrapolate to transform from a tag without
    // seeing that tag.

    // Could not work out pose
    Optional<EstimatedRobotPose> globalPose = this.getGlobalPose();
    if(globalPose.isEmpty()) {
      return Optional.empty();
    }

    // Look for tag
    List<AprilTag> tags = this.aprilTagFieldLayout.getTags();
    for(AprilTag tag : tags) {
      if(tag.ID == tagID) {
        return Optional.of(tag.pose.minus(globalPose.get().estimatedPose));
      }
    }

    // Required tag not on field
    return Optional.empty();
  }
}
