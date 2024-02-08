// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Camera;
// This class will constantly updating the position if it see 2 april tags -Nathan 1/27/24
public class VisionPose extends Command {
  /** Creates a new VisionPose. */
  CommandSwerveDrivetrain swerve;
  Camera camera;

  public VisionPose(CommandSwerveDrivetrain swerve, Camera camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.camera = camera;

    addRequirements(camera);
  }

  public double getVisionDeviation(Optional<EstimatedRobotPose> result) {
    if (!result.isPresent()) {
      return 0;
    }

    EstimatedRobotPose camPose = result.get();

    return swerve.getOdometry().getEstimatedPosition().minus(camPose.estimatedPose.toPose2d()).getTranslation()
        .getNorm();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Just printing stuff for debugging purposes
    SmartDashboard.putNumber("What the bot think it angle is", swerve.getOdometry().getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putNumber("What Path Planner think it angle is", swerve.getState().Pose.getRotation().getDegrees());
    SmartDashboard.putString("VisionPose", "I'm getting called!");
    Optional<EstimatedRobotPose> result = camera
        .getEstimatedGlobalPose(swerve.getOdometry().getEstimatedPosition());

    if (!result.isPresent()) {
      return;
    }

    EstimatedRobotPose camPose = result.get();
    SmartDashboard.putNumber("BotPosX", camPose.estimatedPose.toPose2d().getX());
    SmartDashboard.putNumber("BotPosY", camPose.estimatedPose.toPose2d().getY());
    SmartDashboard.putNumber("BotPosHeading", camPose.estimatedPose.toPose2d().getRotation().getDegrees());
    if ((camPose.estimatedPose.getX() != 0 && camPose.estimatedPose.getY() != 0 &&
    /*
     * Only update if the camera pose is within 1 meter of the estimated position.
     * This will hopefully remove values that are not realistic.
     */
        Math.abs(swerve.getOdometry().getEstimatedPosition().getX() - camPose.estimatedPose.getX()) < 1.0 &&
        Math.abs(swerve.getOdometry().getEstimatedPosition().getY() - camPose.estimatedPose.getY()) < 1.0)
        && !RobotState.isDisabled()&&camera.getLatestResult().targets.size()>1) {
      swerve.setOdometryVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
