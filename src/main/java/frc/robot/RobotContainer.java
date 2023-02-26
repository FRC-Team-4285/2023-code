// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants.AutoConstants;
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
  private final int rotationAxis = 2; //was 4 on Xbox

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
  private JoystickButton btnConeGrab;

  /* Subsystems */
  public final SwerveBase swerveBase;
  public final ClimberArmBase climberArmBase;
  public final IntakeBase intakeBase;
  public final PickupArmBase pickupArmBase;
  public final SuctionCupBase suctionCupBase;

  public Joystick getJoystick(int stickID) {
    if (stickID == 1) return passengerJoystick;
    return driverJoystick; //default stick
  }

  public SwerveBase getSwerveSubsytem() {
    return swerveBase;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driverJoystick = new Joystick(0);
    passengerJoystick = new Joystick(1);

    swerveBase = new SwerveBase();
    swerveBase.setDefaultCommand(
      new TeleopSwerve(
        swerveBase,
        // currently code has no deadband, add deadband here if needed in future
        () -> driverJoystick.getRawAxis(translationAxis),
        () -> driverJoystick.getRawAxis(strafeAxis),
        () -> -driverJoystick.getRawAxis(rotationAxis),
        () -> false //inverted=fieldCentric, non-inverted=RobotCentric
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
    zeroGyro.whileHeld(new InstantCommand(() -> swerveBase.getNavX().reset()));

    // Arm Manual Raise
    btnArmRaise = new JoystickButton(driverJoystick, 6);
    btnArmRaise.whileHeld(new PickupArmUp(pickupArmBase));

    // Arm Manual Lower
    btnArmLower = new JoystickButton(driverJoystick, 5);
    btnArmLower.whileHeld(new PickupArmDown(pickupArmBase));

    // Climber Manual Raise
    btnClimberUp = new JoystickButton(driverJoystick, 3);
    btnClimberUp.whileHeld(new ClimberUp(climberArmBase));

    // Climber Manual Lower
    btnClimberDown = new JoystickButton(driverJoystick, 4);
    btnClimberDown.whileHeld(new ClimberDown(climberArmBase));

    // Intake Manual Raise
    btnIntakeUp = new JoystickButton(driverJoystick, 9);
    btnIntakeUp.whileHeld(new IntakeUp(intakeBase));

    // Intake Manual Lower
    btnIntakeDown = new JoystickButton(driverJoystick, 10);
    btnIntakeDown.whileHeld(new IntakeDown(intakeBase));

    btnSuctionEngage = new JoystickButton(driverJoystick, 11);
    btnSuctionEngage.onTrue(new SuctionCupEngage(suctionCupBase));

    btnSuctionRelease = new JoystickButton(driverJoystick, 12);
    btnSuctionRelease.onTrue(new SuctionCupRelease(suctionCupBase));

    btnConeGrab = new JoystickButton(driverJoystick, 8);
    btnConeGrab.whileHeld(new ConeGrab(pickupArmBase));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public CommandBase getAutonomousCommand() {
    PathPlannerTrajectory path = PathPlanner.loadPath(
      AutoConstants.SELECTED_AUTO,
      new PathConstraints(
        AutoConstants.MAX_SPEED,
        AutoConstants.MAX_ACCELLERATION
      )
    );

    return new SequentialCommandGroup(
      swerveBase.followTrajectoryCommand(path, true)
    );
  }

  // public int getArmRaiseBtnStatus() {
  //   return btnArmRaise.isPressed()
  // }

  // public int getArmLowerBtnStatus() {
  //   return btnArmLower.isPressed()
  // }

  // public int getClimberUpBtnStatus() {
  //   return btnClimberUp.isPressed()
  // }

  // public int getClimberDownBtnStatus() {
  //   return btnClimberDown.isPressed()
  // }

  // public int getIntakeUpBtnStatus() {
  //   return btnIntakeUp.isPressed()
  // }

  // public int getIntakeDownBtnStatus() {
  //   return btnIntakeDown.()
  // }

  // public int getSuctionEngageBtnStatus() {
  //   return btnSuctionEngage.isPressed()
  // }

  // public int getSuctionReleaseBtnStatus() {
  //   return btnSuctionRelease.isPressed()
  // }

}
