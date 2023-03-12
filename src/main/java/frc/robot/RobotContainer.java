// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Swerve;
import frc.robot.commands.*;
import frc.robot.subsystems.ClimberArmBase;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.PickupArmBase;
import frc.robot.subsystems.SuctionArmBase;
import frc.robot.subsystems.SuctionCupBase;
import frc.robot.subsystems.SwerveBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Joystick driverJoystick;
  private final Joystick streamDeck;

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
  private JoystickButton btnFloorIntakeExtend;
  private JoystickButton btnFloorIntakeRetract;
  private JoystickButton btnArmPiston;
  private JoystickButton btnArmStartPos;
  private JoystickButton btnArmDropPos;
  private JoystickButton btnArmFeederPos;
  private JoystickButton btnPickupArmGrab;
  private JoystickButton btnPickupArmRelease;
  private JoystickButton btnFloorIntakeGrab;
  private JoystickButton btnFloorIntakeRelease;
  private JoystickButton btnClimberDownPos;
  private JoystickButton btnClimberUpPos;
  /* Subsystems */
  public final SwerveBase swerveBase;
  public final ClimberArmBase climberArmBase;
  public final IntakeBase intakeBase;
  public final PickupArmBase pickupArmBase;
  public final SuctionCupBase suctionCupBase;
  public final SuctionArmBase suctionArmBase;

  public Joystick getJoystick() {
    return driverJoystick;
  }

  public Joystick getStreamDeck() {
    return streamDeck;
  }

  public SwerveBase getSwerveSubsytem() {
    return swerveBase;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driverJoystick = new Joystick(0);
    streamDeck = new Joystick(1);

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
    suctionArmBase = new SuctionArmBase();

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // Zero Gyro
    zeroGyro = new JoystickButton(driverJoystick, 7); //resets field-centric heading
    zeroGyro.whenPressed(
      new InstantCommand(
          () -> swerveBase.zeroPigeon()
        )
      );

     //Arm Manual Raise
    //btnArmRaise = new JoystickButton(streamDeck, 13);
    // btnArmRaise.whileHeld(new PickupArmUp(pickupArmBase));

    // Arm Manual Lower
     //btnArmLower = new JoystickButton(streamDeck, 14);
    // btnArmLower.whileHeld(new PickupArmDown(pickupArmBase));

    // Arm Manual Piston
     btnArmPiston = new JoystickButton(streamDeck, 9);
     btnArmPiston.whileHeld(new ArmPiston(suctionArmBase));

    // Arm Drop Config
    btnArmDropPos = new JoystickButton(streamDeck, 2);
    btnArmDropPos.whileHeld(new PickupArmDropPos(pickupArmBase, suctionArmBase));

    // Arm Start Config
    btnArmStartPos = new JoystickButton(streamDeck, 6);
    btnArmStartPos.whileHeld(new PickupArmStartPos(pickupArmBase, suctionArmBase));

    // Arm Feeder Pos
    btnArmFeederPos = new JoystickButton(streamDeck, 10);
    btnArmFeederPos.whileHeld(new PickupArmFeederPos(pickupArmBase, suctionArmBase));

    // Climber Manual Raise
    //btnClimberUp = new JoystickButton(streamDeck, 13);
    //btnClimberUp.whileHeld(new ClimberUp(climberArmBase));

    // Climber Manual Lower
    //btnClimberDown = new JoystickButton(streamDeck, 14);
    //btnClimberDown.whileHeld(new ClimberDown(climberArmBase));

    //Climber Up Pos PID
    //btnClimberUpPos = new JoystickButton(streamDeck, 13);
    //btnClimberUpPos.whileHeld(new ClimberUpPos(climberArmBase));

    //CLimber Down Pos PID
    //btnClimberDownPos = new JoystickButton(streamDeck, 14);
    //btnClimberDownPos.whileHeld(new ClimberDownPos(climberArmBase));

    // Intake Raise
    // btnIntakeUp = new JoystickButton(streamDeck, 13);
    // btnIntakeUp.whileHeld(new ConeGrabberIngestUp(intakeBase));

    // Intake Lower
    // btnIntakeDown = new JoystickButton(streamDeck, 14);
    // btnIntakeDown.whileHeld(new ConeGrabberIngestDown(intakeBase));

    // Intake Grab Cone
    // btnIntakeGrabCone = new JoystickButton(streamDeck, 4);
    // btnIntakeGrabCone.whileHeld(new ConeGrabberIngestCone(intakeBase));

    // Intake Start Position
    // btnIntakeStartPos = new JoystickButton(streamDeck, 2);
    // btnIntakeStartPos.whileHeld(new ConeGrabberIngestStartPos(intakeBase));

    // Extend Floor Intake
    btnFloorIntakeExtend = new JoystickButton(streamDeck, 7);
    btnFloorIntakeExtend.whileHeld(new FloorIntakeExtend(intakeBase));

    // Retract Floor Intake
    btnFloorIntakeRetract = new JoystickButton(streamDeck, 3);
    btnFloorIntakeRetract.whileHeld(new FloorIntakeRetract(intakeBase));

    //Climber Engage
    btnSuctionEngage = new JoystickButton(streamDeck, 15);
    btnSuctionEngage.onTrue(new SuctionCupEngage(suctionCupBase));

    //Suction Release
     btnSuctionRelease = new JoystickButton(streamDeck, 16);
     btnSuctionRelease.onTrue(new SuctionCupRelease(suctionCupBase));

    // Pickup Arm Grab
    btnPickupArmGrab = new JoystickButton(streamDeck, 5);
    btnPickupArmGrab.whileHeld(new PickupArmGrab(suctionArmBase));

    // Pickup Arm Release
    btnPickupArmRelease = new JoystickButton(streamDeck, 1);
    btnPickupArmRelease.whileHeld(new PickupArmRelease(suctionArmBase));

    // Floor Intake Grab
    btnFloorIntakeGrab = new JoystickButton(streamDeck, 4);
    btnFloorIntakeGrab.whileHeld(new FloorIntakeGrab(intakeBase));

    // Floor Intake Release
    btnFloorIntakeRelease = new JoystickButton(streamDeck, 8);
    btnFloorIntakeRelease.whileHeld(new FloorIntakeRelease(intakeBase));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public CommandBase getAutonomousCommand() {
    PathPlannerTrajectory path_a = PathPlanner.loadPath(
      "New New Path",
      new PathConstraints(
        AutoConstants.MAX_SPEED,
        AutoConstants.MAX_ACCELLERATION
      )
    );

    PathPlannerTrajectory path_b = PathPlanner.loadPath(
      "A_AutoClimbCubeCone_2",
      new PathConstraints(
        AutoConstants.MAX_SPEED,
        AutoConstants.MAX_ACCELLERATION
      )
    );


    // How to chain link commands SEQUENTIALLY.
    return new SequentialCommandGroup(
      swerveBase.followTrajectoryCommand(path_a, true)
      //swerveBase.followTrajectoryCommand(path_b, false)
    );

  }

  public boolean getZeroGyroBtnStatus() {
    return zeroGyro.getAsBoolean();
  }

  public boolean getArmRaiseBtnStatus() {
    return btnArmRaise.getAsBoolean();
  }

  public boolean getArmLowerBtnStatus() {
    return btnArmLower.getAsBoolean();
  }

  public boolean getClimberUpBtnStatus() {
    return btnClimberUp.getAsBoolean();
  }

  public boolean getClimberDownBtnStatus() {
    return btnClimberDown.getAsBoolean();
  }

  public boolean getIntakeUpBtnStatus() {
    return btnIntakeUp.getAsBoolean();
  }

  public boolean getIntakeDownBtnStatus() {
    return btnIntakeDown.getAsBoolean();
  }

  public boolean getSuctionEngageBtnStatus() {
    return btnSuctionEngage.getAsBoolean();
  }

  public boolean getSuctionReleaseBtnStatus() {
    return btnSuctionRelease.getAsBoolean();
  }

  public boolean getConeGrabBtnStatus() {
    return btnConeGrab.getAsBoolean();
  }

}
