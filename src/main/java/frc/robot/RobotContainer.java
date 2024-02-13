// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AutoShoot;
import frc.robot.Commands.ShooterCommand;
import frc.robot.Commands.SimpleTwoPieceAuton;
import frc.robot.Commands.VisionPose;
import frc.robot.Subsystems.Camera;
import frc.robot.Subsystems.Shooter;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed

  // Replace all instances of Nathan Speed with MaxSpeed for production code
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController drivercontroller = new CommandXboxController(0); // drive controller
  private final CommandXboxController operatercontroller = new CommandXboxController(1); // operator controller
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Camera m_camera;
  private final Shooter m_shooter;
  private final AutoShoot m_autoshoot;
  //private final VisionPose m_visionpose;

  // Field centric drive
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  // This is the auto that we run open the path planner app for mor details
  private Command runAuto = drivetrain.getAutoPath("April Tag Auton");

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() -> drive
        .withVelocityX(-drivercontroller.getLeftY() * Constants.NathanSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(-drivercontroller.getLeftX() * Constants.NathanSpeed) // Drive left with negative X (left)
        .withRotationalRate(-drivercontroller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
      )
    );

    operatercontroller.leftBumper().whileTrue(drivetrain.applyRequest(() -> 
        // Drive forward with negative Y (forward)
        drive.withVelocityX(-drivercontroller.getLeftY() * Constants.NathanSpeed)

          // Drive left with negative X (left)
          .withVelocityY(-drivercontroller.getLeftX() * Constants.NathanSpeed)

          // Drive counterclockwise with negative X (left)
          .withRotationalRate(m_autoshoot.targetAll())
    ));

    m_shooter.setDefaultCommand(new ShooterCommand(m_shooter, () -> operatercontroller.getLeftY(), () -> operatercontroller.b().getAsBoolean(), () -> operatercontroller.y().getAsBoolean()));

    drivercontroller.a().whileTrue(
      drivetrain.applyRequest(() -> brake)
    );

    drivercontroller.b().whileTrue(
      drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-drivercontroller.getLeftY(), -drivercontroller.getLeftX())))
    );

    operatercontroller.a().whileTrue(Commands.run(() -> m_shooter.setShooterVelocity()));
    operatercontroller.a().whileFalse(Commands.run(() -> m_shooter.stopShooter()));
    
    operatercontroller.x().whileTrue(Commands.run(() -> m_shooter.feedMotorPower(0.6)));
    operatercontroller.x().whileFalse(Commands.run(() -> m_shooter.feedMotorPower(0)));
    
    operatercontroller.leftBumper().whileFalse(Commands.run(() -> m_shooter.targetAngle = 190));

    // reset the field-centric heading on left bumper press
    drivercontroller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    m_shooter = new Shooter();
    m_camera = new Camera(drivetrain);
    m_autoshoot = new AutoShoot(drivetrain, m_shooter, m_camera, drive);
    
    // m_visionpose = new VisionPose(drivetrain, m_camera);
    // m_camera.setDefaultCommand(m_visionpose);

    configureBindings();
  }

  public Command getAutonomousCommand() {
    // return new SimpleTwoPieceAuton(m_shooter, m_autoshoot, runAuto);
    return runAuto;
  }
}
