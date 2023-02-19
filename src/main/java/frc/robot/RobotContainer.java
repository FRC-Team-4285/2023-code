// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.commands.*;
import frc.robot.subsystems.ClimberArmBase;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.PickupArmBase;
import frc.robot.subsystems.SuctionCupBase;
import frc.robot.subsystems.SwerveBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Joystick driverJoystick;
  private final Joystick passengerJoystick;

  /* Drive Controls */
  private final int translationAxis = 1;
  private final int strafeAxis = 0;
  private final int rotationAxis = 3; //flipper axis ranges from 1 to -1

  private final DoubleSupplier rotationSupplier;

  /* Drive Buttons */
  private JoystickButton zeroGyro;
  private JoystickButton btnArmRaise;
  private JoystickButton btnArmLower;
  private JoystickButton btnClimberUp;
  private JoystickButton btnClimberDown;
  private JoystickButton btnIntakeUp;
  private JoystickButton btnIntakeDown;
  private JoystickButton btnSuctionEngage;
  private JoystickButton btnSuctionRelease;

  /* Subsystems */
  private final SwerveBase swerveBase;
  private final ClimberArmBase climberArmBase;
  private final IntakeBase intakeBase;
  private final PickupArmBase pickupArmBase;
  private final SuctionCupBase suctionCupBase;

  public Joystick getDriverJoystick(int stickID) {
    return driverJoystick;
  }
  public Joystick getPassengerJoystick(){
    return passengerJoystick;
  }

  public SwerveBase getSwerveSubsytem() {
    return swerveBase;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driverJoystick = new Joystick(0);
    passengerJoystick = new Joystick(1);

    rotationSupplier = () -> (
      (1-driverJoystick.getRawAxis(rotationAxis))/2.0 *( //flipper axis controls power of rotation
        (driverJoystick.getRawButton(11) ? -1.0:0.0) //button 11 turns left/  - direction
      + (driverJoystick.getRawButton(12) ? 1.0:0.0)  //button 12 turns right/ + direction
    ));

    swerveBase = new SwerveBase();
    swerveBase.setDefaultCommand(
      new TeleopSwerve(
        swerveBase,
        //swerve takes in doubleSuppliers for translation, strafing, and rotation
        () -> driverJoystick.getRawAxis(translationAxis),
        () -> driverJoystick.getRawAxis(strafeAxis),
        rotationSupplier, //see above
        () -> !driverJoystick.getRawButton(1) //field-centric by default
        )
    );

    climberArmBase = new ClimberArmBase();
    intakeBase = new IntakeBase();
    pickupArmBase = new PickupArmBase();
    suctionCupBase = new SuctionCupBase();  

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // Zero Gyro
    zeroGyro = new JoystickButton(driverJoystick, 7); //resets field-centric heading
    zeroGyro.onTrue(new InstantCommand(() -> swerveBase.getPigeonSensor().reset()));

    // Arm Manual Raise
    btnArmRaise = new JoystickButton(driverJoystick, 6);
    btnArmRaise.onTrue(new ArmUp(pickupArmBase));

    // Arm Manual Lower
    btnArmLower = new JoystickButton(driverJoystick, 5);
    btnArmLower.onTrue(new ArmDown(pickupArmBase));

    // Climber Manual Raise
    btnClimberUp = new JoystickButton(driverJoystick, 3);
    btnClimberUp.onTrue(new ClimberUp(climberArmBase));

    // Climber Manual Lower
    btnClimberDown = new JoystickButton(driverJoystick, 4);
    btnClimberDown.onTrue(new ClimberDown(climberArmBase));

    // Intake Manual Raise
    btnIntakeUp = new JoystickButton(driverJoystick, 5);
    btnIntakeUp.onTrue(new IntakeUp(intakeBase));

    // Intake Manual Lower
    btnIntakeDown = new JoystickButton(driverJoystick, 6);
    btnIntakeDown.onTrue(new IntakeDown(intakeBase));

    btnSuctionEngage = new JoystickButton(driverJoystick, 7);
    btnSuctionEngage.onTrue(new SuctionCupEngage(suctionCupBase));

    btnSuctionRelease = new JoystickButton(driverJoystick, 8);
    btnSuctionRelease.onTrue(new SuctionCupRelease(suctionCupBase));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public CommandBase getAutonomousCommand() {
    PathPlannerTrajectory path = PathPlanner.loadPath(
      "ExamplePath",
      new PathConstraints(
        3,
        2
      )
    );

    return new SequentialCommandGroup(
      swerveBase.followTrajectoryCommand(path, true)
    );
  }
}
