// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Swerve {

    public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.1, 0.15, 0.01);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(19.5);
    public static final double wheelBase = Units.inchesToMeters(16.5);

    // nominal (real) divided by fudge factor
    public static final double wheelDiameter = Units.inchesToMeters(4.0 / 1.04085);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = ((150.0 / 7.0) / 1.0); // 150/7:1

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                    new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // front left
                    new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right
                    new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // rear left
                    new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // rear right
    );

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = 11.5;

    public static final int frontLeftRotationMotorId = 8;
    public static final int frontLeftDriveMotorId = 7;

    public static final int frontRightRotationMotorId = 6;
    public static final int frontRightDriveMotorId = 5;

    public static final int rearLeftRotationMotorId = 2;
    public static final int rearLeftDriveMotorId = 1;

    public static final int rearRightRotationMotorId = 4;
    public static final int rearRightDriveMotorId = 3;

    public static final int frontLeftRotationEncoderId = 4;
    public static final int frontRightRotationEncoderId = 1;
    public static final int rearLeftRotationEncoderId = 3;
    public static final int rearRightRotationEncoderId = 2;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = 7.5 / 4.0;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 3.5;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double cameraToFrontEdgeDistanceMeters = Units.inchesToMeters(7);

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
