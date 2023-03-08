// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double yDeadband = 0.1;
    public static final double xDeadband = 0.1;
  }

  public static class DriveConstants {
    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 2;
    public static final int kRightMotor1Port = 3;
    public static final int kRightMotor2Port = 4;

    public static final double gyroOffset = 0;

    public static final double trackWidth = 0.61;

    public static final double gearRatio = 1 / 10.71;
    public static final double wheelRatio = 0.478779;
  }

  public static class MotionConstants {
    public static final double ksVolts = 0.11233;
    public static final double kvVoltSecondsPerMeter = 1.3694;
    public static final double kaVoltSecondsSquaredPerMeter = 0.16351;
    public static final double kPDriveVel = 0; // 0.65605

    public static final double maxVelocity = 1.0;
    public static final double maxAcceleration = 0.5;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double ramseteB = 2;
    public static final double ramseteZeta = 0.7;
  }

  public static class Vision {
    public static final double goalHeightInches = 30;
    public static final double cameraHeightInches = 26.5;
    public static final double cameraAngleDegrees = 0;
  }

  public static class IntakeConstants {
    public static final int intakeMotorPortL = 6;
    public static final int intakeMotorPortR = 7;
  }

}
