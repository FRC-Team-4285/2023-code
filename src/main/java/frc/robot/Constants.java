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
    public static final double trackWidth = Units.inchesToMeters(21.5); //measured from center of each module
    public static final double wheelBase = Units.inchesToMeters(21.5);

    public static final double wheelDiameter = Units.inchesToMeters(4.0*1.0);
    /*TODO: Adjust wheelDiameter based on measurements of how far robot actually moves*/
    public static final double wheelCircumference = wheelDiameter * Math.PI;
/*
    public static final double driveGearRatio = 8.16; // Mk3 Standard drive ratio 
    public static final double angleGearRatio = 12.8; // Mk3 Standard steer ratio
*/

    public static final double driveGearRatio = 5.14; // Mk4 L4 drive ratio
    public static final double angleGearRatio = 12.8; // Mk4 L4 steer ratio

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                    new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // front left, ++ quadrant
                    new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right, +- quadrant
                    new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // rear left, -+ quadrant
                    new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // rear right, -- quadrant
    );

    /* Swerve Profiling Values */
    public static final double maxSpeed = 1.0; // was 1.3, robot gets faster if this is lower???
    public static final double maxAngularVelocity = 3.0; //9.554 max theoretical

    public static final int frontLeftRotationMotorId = 7; //was 8
    public static final int frontLeftDriveMotorId = 8; //was 23

    public static final int frontRightRotationMotorId = 1; //was 5
    public static final int frontRightDriveMotorId = 2; //was 22

    public static final int rearLeftRotationMotorId = 5; //was 2
    public static final int rearLeftDriveMotorId = 6; //was 1

    public static final int rearRightRotationMotorId = 3; //was 4
    public static final int rearRightDriveMotorId = 4; //was 3

    public static final int frontLeftRotationEncoderId = 4;
    public static final int frontRightRotationEncoderId = 1;
    public static final int rearLeftRotationEncoderId = 3;
    public static final int rearRightRotationEncoderId = 2;

    /* These Constants set the max speed that will be requested by the Teleop Command */
    public static final double kTeleDriveMaxSpeedMetersPerSecond = 3.688;
    /* SDS states that neo-driven standard ratio Mk3 modules have a free speed of 3.688 m/s */
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 3.0;//9.554 max theoretical
    /* calculated from max drive speed divided by distance of Mk3 module from center of robot */
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double cameraToFrontEdgeDistanceMeters = Units.inchesToMeters(7);

    public static final int PIGEON_SENSOR_ID = 0;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    //TODO: PATHPLANNER
  }

  public static final class ArmConstants {
    // Encoder IDs
    public static final int ARM_MOTOR_ID = 9;

    // Motor Speed
    public static final double ARM_MOTOR_POWER = 2.5;    

    // Direction
    public static final boolean ARM_DIRECTION_UP = true;
    public static final boolean ARM_DIRECTION_DOWN = false;

    // PID Positions
    /*These values are only accurate if robot is turned on in starting configuration*/
    public static final double FEEDER_POS = 0.265;
    public static final double START_POS = 0.00;
    public static final double START_PLUS_POS = -0.01;
    public static final double DROP_POS = -0.674;
  }

  public static final class ClimberConstants {
    // Encoder IDs
    public static final int CLIMBER_MOTOR_LEFT_ID = 10;
    public static final int CLIMBER_MOTOR_RIGHT_ID = 11;

    public static final int BATTERY_MOTOR_ID = 12;

    // Motor Speed
    public static final double CLIMBER_MOTOR_POWER = 5.5;

    // Direction
    public static final boolean CLIMBER_DIRECTION_UP = true;
    public static final boolean CLIMBER_DIRECTION_DOWN = false;

    // PID Positions
    public static final double CLIMBER_POS_DOWN_SUCTION = -119.0;
    public static final double CLIMBER_POS_DOWN = -0.1;
    public static final double CLIMBER_POS_UP = -9.25;
  }

  public static final class IntakeConstants {
    // Encoder IDs
    public static final int INTAKE_MOTOR_ID = 13;

    // Motor Speed
    public static final double INTAKE_MOTOR_POWER = 0.1;

    // Direction
    public static final boolean INTAKE_DIRECTION_UP = true;
    public static final boolean INTAKE_DIRECTION_DOWN = false;

    public static final double INTAKE_EXTEND_POS = 54.97;
    public static final double INTAKE_RETRACT_POS = 0.0;
  }

  public static final class SuctionConstants {
    // Action
    public static final boolean SUCTION_CUP_ENGAGE = true;
    public static final boolean SUCTION_CUP_RELEASE = false;

    public static final int PUMP_READY = 0;
    public static final int PUMP_NEED_RESET = 1;
    public static final int PUMP_WAIT = 2;
    public static final int PUMP_COMPLETE = 3;

    public static final double PUMP_DELAY_MS = 125.0;
  }

  public static class HardwareCAN {
    // Hardware CAN IDs
    public static final int PDU_PUMP = 20;
    public static final int PNEUMATIC_HUB = 21;
  }

  public static class LEDConstants {
    // DIO Port IDs for LEDs.
    public static final int DIO_LED_STRIP = 2;
  }

  public static class PneumaticChannels {
    public static final int CLIMBER_ARM_LIFTER = 0; //lifts robot when suction cup is engaged
    public static final int ARM_LOCKER_OFF = 1; //cone arm is held up @ start of match, must be disengaged after match start
    public static final int ARM_LOCKER_ON = 7;
    public static final int CUP_PUMPER = 2; //pumps up suction cup, needs at least 6-7 pumps before robot can support itself
    public static final int CUP_RELEASE_OFF = 3; //releases suction cup
    public static final int CUP_RELEASE_ON = 4;
    public static final int CUBE_GRAB = 5; //cone arm main grabber, used on its own for cubes
    public static final int CONE_GRAB = 6; //cone arm auxilliary grabber, used alongside CUBE_GRAB to grab cones better
    public static final int FLOOR_EXTEND_OFF = 8; //extends grabber for grabbing cones off floor
    public static final int FLOOR_EXTEND_ON = 9;
    public static final int FLOOR_GRAB_OFF = 10; //actuates grabber for grabbing cones off floor
    public static final int FLOOR_GRAB_ON = 11;
  }
}