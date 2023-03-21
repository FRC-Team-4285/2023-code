package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveBase;

public class LimelightTrackDrive extends CommandBase {
  /*
   * Limelight Tracking Drive Command
   */

   private double startTime = 0.0;
   private final SwerveBase drive;
   private NetworkTable table;
   private NetworkTableEntry ty;

    public LimelightTrackDrive(SwerveBase swerveBase) {
        drive = swerveBase;

        table = NetworkTableInstance.getDefault().getTable("limelight");
        ty = table.getEntry("ty");

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

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        System.out.println(ty.getDouble(0.0));
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
