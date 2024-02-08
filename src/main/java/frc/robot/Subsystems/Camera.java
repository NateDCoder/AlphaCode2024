// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Handle the camera and vision pipeline 1/27/24 -Nathan
public class Camera extends SubsystemBase {
  PhotonCamera camera;
  PhotonPoseEstimator photonPoseEstimator;
  /** Creates a new Camera. */
  public Camera() {
    camera = new PhotonCamera("Arducam_AprilTags");
    Transform3d robotToCam = new Transform3d(new Translation3d(.35, 0.08, .18), new Rotation3d(Math.toDegrees(0), Math.toRadians(-30),0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

  }
  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }
  public boolean checkTargetExistence(PhotonPipelineResult result) {
    return result.hasTargets();
  }
  public List<PhotonTrackedTarget> getTargets(PhotonPipelineResult result) {
    return result.getTargets();
  }
  public PhotonTrackedTarget getBesTarget(PhotonPipelineResult result) {
    return result.getBestTarget();
  }
  public Transform3d getTag3D(PhotonTrackedTarget target) {
    return target.getBestCameraToTarget();
  }
  public double getLatency() {
    return getLatestResult().getLatencyMillis() / 1000.0;
  }
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    SmartDashboard.putNumber("Amount of tags detected", getLatestResult().targets.size());
    return photonPoseEstimator.update();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
