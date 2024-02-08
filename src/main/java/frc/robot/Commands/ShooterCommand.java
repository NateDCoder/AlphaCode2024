// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Shooter;

public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
  Shooter shooter;
  Supplier<Double> leftYSupplier;
  Supplier<Boolean> bButtonSupplier;
  Supplier<Boolean> yButtonSupplier;
  double intakePower = 1000;
  public ShooterCommand(Shooter shooter, Supplier<Double> leftYSupplier, Supplier<Boolean> bButtonSupplier,
      Supplier<Boolean> yButtonSupplier) {
    this.shooter = shooter;
    this.leftYSupplier = leftYSupplier;
    this.bButtonSupplier = bButtonSupplier;
    this.yButtonSupplier = yButtonSupplier;

    addRequirements(shooter);
    SmartDashboard.putNumber("Intake Power", intakePower);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double intakePowerValue = SmartDashboard.getNumber("Intake Power", 0);
    // shooter.pivotRawPower(-leftYSupplier.get());
    if(intakePowerValue != intakePower) {
      intakePower = intakePowerValue;
    }
    shooter.pivotRawPower();
    if (yButtonSupplier.get()) {
      shooter.intakeMotorPower(-intakePower);
    } else if (bButtonSupplier.get()) {
      shooter.intakeMotorPower(intakePower);
    } else {
      shooter.intakeMotorPower(0);
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
