// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleTwoPieceAuton extends SequentialCommandGroup {
  /** Creates a new SimpleTwoPieceAuton. */
  public SimpleTwoPieceAuton(Shooter m_shooter, AutoShoot m_autoshoot, Command path) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.parallel(
            Commands.run(() -> m_shooter.setShooterVelocity()),
            Commands.run(() -> m_shooter.feedMotorPower(0.6)),
            Commands.run(() -> m_shooter.targetAngle = m_autoshoot.metersToPivotAngle(7)[0]),
            Commands.sequence(
                new WaitCommand(0.5),
                Commands.parallel(
                    Commands.run(() -> m_shooter.intakeMotorPower(-1000)),
                    Commands.sequence(
                        new WaitCommand(3),
                        path)))));
  }
}
