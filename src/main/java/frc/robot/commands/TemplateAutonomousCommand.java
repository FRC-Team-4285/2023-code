package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveBase;

public class TemplateAutonomousCommand extends CommandBase {
  /*
   * Autonomous Command
   * ------------------
   * 
   * This command is a stub that runs during autonomous.
   * Currently unused, will be filled soon.
   */

   private RobotContainer robotContainer;
   private double startTime = 0.0;
   private final SwerveBase drive;

    public TemplateAutonomousCommand(SwerveBase swerveBase) {
        drive = swerveBase;
        addRequirements(swerveBase);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        startTime = getCurrentTime();
        drive.zeroPigeon();
    }

    private double getCurrentTime() {
        /*
        * Returns current time in milliseconds.
        */

        return System.currentTimeMillis();
    }

    private double getTimeSinceInitialized() {
        return getCurrentTime() - startTime;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        if (getTimeSinceInitialized() < 15000) { // less than 15 seconds
            drive.drive(0.3, 0, 0, true);
        }
        else if (getTimeSinceInitialized() < 25000) { // greater than 15 and less than 25.
            drive.drive(-0.3, 0, 0, true);
        }
        else { // over 25s
            drive.drive(0.0, 0, 0, true);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean isInterrupted) {
    }
}
