// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.SwerveBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final Joystick driver;

  /* Drive Controls */
  private final int translationAxis = 1;
  private final int strafeAxis = 0;
  private final int rotationAxis = 2; //was 4 on Xbox

  /* Drive Buttons */
  private final JoystickButton zeroGyro;

  /* Subsystems */
  private final SwerveBase swerveBase;

  public Joystick getDriver() {
    return driver;
  }

  public SwerveBase getSwerveSubsytem() {
    return swerveBase;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driver = new Joystick(0);
    zeroGyro = new JoystickButton(driver, 7); //resets field-centric heading
    swerveBase = new SwerveBase();
    swerveBase.setDefaultCommand(
      new TeleopSwerve(
        swerveBase,
        // currently code has no deadband, add deadband here if needed in future
        () -> driver.getRawAxis(translationAxis),
        () -> driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis),
        () -> !driver.getRawButton(1) //inverted=fieldCentric, non-inverted=RobotCentric
      )
    );

    HashMap<String, Command> eventMap = new HashMap<>();

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      swerveBase::getPose, // Pose2d supplier
      swerveBase::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      swerveBase.getKinematics(), // SwerveDriveKinematics
      new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      swerveBase::setModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      swerveBase // The drive subsystem. Used to properly set the requirements of path following commands
    );

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    zeroGyro.onTrue(new InstantCommand(() -> swerveBase.getPigeonSensor().reset()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public CommandBase getAutonomousCommand() {
    // An example command will be run in autonomous
    PathPlannerTrajectory path = PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));
    return new SequentialCommandGroup(
      swerveBase.followTrajectoryCommand(path, true)
    );
  }
}
