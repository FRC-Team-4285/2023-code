package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.PickupArmBase;
import frc.robot.subsystems.SuctionArmBase;
import frc.robot.subsystems.SwerveBase;

public class AutoDropCube extends CommandBase {
  /*
   * Autonomous Command
   * ------------------
   * 
   * This command is a stub that runs during autonomous.
   * Currently unused, will be filled soon.
   */

   private double startTime = 0.0;
   private final SwerveBase drive;
   private final PickupArmBase armBase;
   private final SuctionArmBase armBaseCone;

    public AutoDropCube(SwerveBase swerveBase, PickupArmBase pickupArmBase, SuctionArmBase suctionArmBase) {
        drive = swerveBase;
        armBase = pickupArmBase;
        armBaseCone = suctionArmBase;
        addRequirements(swerveBase, armBaseCone);
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
        double timeSinceInitialized = getTimeSinceInitialized();

        // Do not move ever.
        drive.drive(0.0, 0.0, 0.0, true);

        if (timeSinceInitialized < 2000) {
            armBaseCone.unlock_arm();
            armBase.go_to_position(ArmConstants.DROP_POS);
        }
        else if (timeSinceInitialized < 2300) {
            armBaseCone.release_cone();
            armBaseCone.release_cube();
        }
        else {
            armBase.go_to_position(ArmConstants.START_POS);
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
