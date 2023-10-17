// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import java.util.function.DoubleSupplier;
import java.util.function.BiFunction;
import java.lang.Math;

import frc.robot.Constants.AutoConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.ClimberArmBase;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.PickupArmBase;
import frc.robot.subsystems.SuctionArmBase;
import frc.robot.subsystems.SwerveBase;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private static final boolean WANT_MANUAL_CLIMBER = false;
  // The robot's subsystems and commands are defined here...
  private final Joystick driverJoystick;
  private final Joystick streamDeck;
  private SendableChooser<CommandBase> auto_chooser;

  /* Drive Controls */
  private final int translationAxis = 1;
  private final int strafeAxis = 0;
  private final int rotationAxis = 2; //was 4 on Xbox
  private final int sliderAxis = 3;

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
  private JoystickButton btnJiggle;
  private JoystickButton btnArmStartPos;
  private JoystickButton btnArmDropPos;
  private JoystickButton btnArmFeederPos;
  private JoystickButton btnPickupArmGrab;
  private JoystickButton btnPickupArmRelease;
  private JoystickButton btnFloorIntakeGrab;
  private JoystickButton btnFloorIntakeRelease;
  private JoystickButton btnCubeGrabLight;
  private JoystickButton btnConeGrabLight;
  private JoystickButton btnLimelightTrackDrive;

  /* Subsystems */
  public final SwerveBase swerveBase;
  /*
  public final ClimberArmBase climberArmBase;
  public final IntakeBase intakeBase;
  public final PickupArmBase pickupArmBase;
  public final SuctionArmBase suctionArmBase;
  */

  /* LEDs */
  public final DigitalOutput ledCubeIndicator;
  public final DigitalOutput ledConeIndicator;
  public final DigitalOutput ledBlueAlliance;
  public final DigitalOutput ledRedAlliance;

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
    DoubleSupplier limit = () -> 0.55 - 0.45*driverJoystick.getRawAxis(sliderAxis);
    /*maps sliderAxis to be between 0.1 and 1.0*/
    DoubleSupplier stopRotation = () -> driverJoystick.getRawButton(12) ? 0.0 : 1.0;
    /* clamps rotation to zero if button 12 is pressed */
    BiFunction<Double, Double, Double> Clamp = (val,lim) -> (Math.abs(val) < lim) ? val : Math.copySign(lim,val);
    /*clamps value to be within a certain limit, also preserves sign */

    swerveBase = new SwerveBase();
    swerveBase.setDefaultCommand(
      new TeleopSwerve(
        swerveBase,
        () -> Clamp.apply(driverJoystick.getRawAxis(translationAxis), limit.getAsDouble()),
        () -> Clamp.apply(driverJoystick.getRawAxis(strafeAxis), limit.getAsDouble()),
        () -> -Clamp.apply(driverJoystick.getRawAxis(rotationAxis), limit.getAsDouble()*stopRotation.getAsDouble()),
        () -> !driverJoystick.getRawButton(1) //inverted=fieldCentric, non-inverted=RobotCentric
      )
    );
    
    ledCubeIndicator = new DigitalOutput(2);
    ledConeIndicator = new DigitalOutput(3);
    ledBlueAlliance = new DigitalOutput(4);
    ledRedAlliance = new DigitalOutput(5);
/*
    climberArmBase = new ClimberArmBase(this);
    intakeBase = new IntakeBase(this);
    pickupArmBase = new PickupArmBase(this);
    suctionArmBase = new SuctionArmBase(this);
*/
    //Create & Configue AutoChooser
    /*
    auto_chooser = new SendableChooser<CommandBase>();
    auto_chooser.addOption("Auto B", new AutoBDropCubeOnBalancePID(swerveBase, pickupArmBase, suctionArmBase));
    auto_chooser.addOption("Blue A", new AutoBlueADropCubeOutCommunity(swerveBase, pickupArmBase, suctionArmBase));
    auto_chooser.addOption("Blue C", new AutoBlueCDropCubeOutCommunity(swerveBase, pickupArmBase, suctionArmBase));
    auto_chooser.addOption("Red A", new AutoRedADropCubeOutCommunity(swerveBase, pickupArmBase, suctionArmBase));
    auto_chooser.addOption("Red C", new AutoRedCDropCubeOutCommunity(swerveBase, pickupArmBase, suctionArmBase));
    auto_chooser.setDefaultOption("Auto B", new AutoBDropCubeOnBalancePID(swerveBase, pickupArmBase, suctionArmBase));
    SmartDashboard.putData("Auto Paths", auto_chooser);
    */
    
    // Configure the trigger bindings
    configureBindings();
    configureLights();
  }

  private void configureLights() {
    ledCubeIndicator.set(true);
    ledConeIndicator.set(true);
    ledBlueAlliance.set(true);
    ledRedAlliance.set(true);
  }

  private void configureBindings() {
    // Zero Gyro
    zeroGyro = new JoystickButton(driverJoystick, 7); //resets field-centric heading
    zeroGyro.whenPressed(
      new InstantCommand(
          () -> swerveBase.zeroPigeon()
        )
      );
  }
}
    // Jiggle
    /*
    btnJiggle = new JoystickButton(streamDeck, 9);
    btnJiggle.whileHeld(new Jiggle(suctionArmBase));

    boolean WANT_MANUAL_ARM = false;

    if (!WANT_MANUAL_ARM) {
      // Arm Drop Config
      btnArmDropPos = new JoystickButton(streamDeck, 2);
      btnArmDropPos.onTrue(new PickupArmDropPos(pickupArmBase, suctionArmBase, this));

      // Arm Start Config
      btnArmStartPos = new JoystickButton(streamDeck, 6);
      btnArmStartPos.onTrue(new PickupArmStartPos(pickupArmBase, suctionArmBase, this));

      // Arm Feeder Pos
      btnArmFeederPos = new JoystickButton(streamDeck, 10);
      btnArmFeederPos.onTrue(new PickupArmFeederPos(pickupArmBase, suctionArmBase, this)); 
    }
    else {
      //Arm Manual Raise
      btnArmRaise = new JoystickButton(streamDeck, 6);
      btnArmRaise.whileHeld(new PickupArmUp(pickupArmBase));

      // Arm Manual Lower
      btnArmLower = new JoystickButton(streamDeck, 10);
      btnArmLower.whileHeld(new PickupArmDown(pickupArmBase));
    }

    // Climber Manual Raise
    btnClimberUp = new JoystickButton(streamDeck, 13);
    btnClimberUp.whileHeld(new ClimberUp(climberArmBase));

    // Climber Manual Lower
    btnClimberDown = new JoystickButton(streamDeck, 14);
    btnClimberDown.whileHeld(new ClimberDown(climberArmBase));

    // Grab Cone Light Indicator
    btnConeGrabLight = new JoystickButton(streamDeck, 11);
    btnConeGrabLight.whileHeld(new ConeGrabLight(suctionArmBase));

    // Grab Cube Light Indicator
    btnCubeGrabLight = new JoystickButton(streamDeck, 12);
    btnCubeGrabLight.whileHeld(new CubeGrabLight(suctionArmBase));
*/
    // Climber Arm Suction Engage (MOVES CLIMBER ARM + ENGAGES)
    // btnClimberSuctionEngage = new JoystickButton(streamDeck, 15);
    // btnClimberSuctionEngage.whileHeld(new ClimberSuctionEngagePos(climberArmBase, suctionCupBase));

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
/*
    // Extend Floor Intake
    btnFloorIntakeExtend = new JoystickButton(streamDeck, 7);
    btnFloorIntakeExtend.whileHeld(new FloorIntakeExtend(intakeBase));

    // Retract Floor Intake
    btnFloorIntakeRetract = new JoystickButton(streamDeck, 3);
    btnFloorIntakeRetract.whileHeld(new FloorIntakeRetract(intakeBase));

    // //Climber Engage
    // btnSuctionEngage = new JoystickButton(streamDeck, 15);
    // btnSuctionEngage.onTrue(new SuctionCupEngage(suctionCupBase));

    // //Suction Release
    // btnSuctionRelease = new JoystickButton(streamDeck, 16);
    // btnSuctionRelease.onTrue(new SuctionCupRelease(suctionCupBase));

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

    // Limelight Track Drive
    btnLimelightTrackDrive = new JoystickButton(driverJoystick, 11);
    btnLimelightTrackDrive.whileHeld(new LimelightTrackDrive(swerveBase));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
  public CommandBase getAutonomousCommand() {
    
    CommandBase autonomousCommand = new AutoBlueDropCubeGrabCone(
      swerveBase,
      pickupArmBase,
      suctionArmBase,
      intakeBase
    );

    return autonomousCommand;
    
    //return auto_chooser.getSelected();
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

  public boolean getArmDropStatus() {
    return btnArmDropPos.getAsBoolean();
  }

  public boolean getArmFeedStatus () {
    return btnArmFeederPos.getAsBoolean();
  }

  public boolean getArmStartStatus () {
    return btnArmStartPos.getAsBoolean();
  }

}
*/