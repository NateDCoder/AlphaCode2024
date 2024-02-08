// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static PIDFF_CONSTANTS shooterPID = new PIDFF_CONSTANTS(6e-5, 0, 0, 0.000180);

  public static final int BOTTOM_SHOOTER_MOTOR_ID = 13;
  public static final int TOP_SHOOTER_MOTOR_ID = 14;
  public static final int FEED_MOTOR_ID = 15;
  public static final int PIVOT_MOTOR_ID = 16;
  public static final int INTAKE_MOTOR_ID = 17;

  public static final double INTAKE_POWER = 0.5;

  public static final double PIVOT_ANGLE_OFFSET = 45;

  public static class PIDFF_CONSTANTS {
    public double p, i, d, ff;

    PIDFF_CONSTANTS(double p, double i, double d, double ff) {
      this.p = p;
      this.i = i;
      this.d = d;
      this.ff = ff;
    }

    public double getP() {
      return p;
    }

    public double getI() {
      return i;
    }

    public double getD() {
      return d;
    }

    public double getFF() {
      return ff;
    }
  }
}
