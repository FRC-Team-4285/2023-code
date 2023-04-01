package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.PickupArmBase;
import frc.robot.subsystems.SuctionArmBase;
import frc.robot.subsystems.SwerveBase;

public class AutoRedADropCubeOutCommunity extends CommandBase {
  /*
   * Autonomous Command
   * ------------------
   */

   private double startTime = 0.0;
   private final SwerveBase drive;
   private final PickupArmBase armBase;
   private final SuctionArmBase armBaseCone;

   public AutoRedADropCubeOutCommunity(SwerveBase swerveBase, PickupArmBase pickupArmBase, SuctionArmBase suctionArmBase) {
        drive = swerveBase;
        armBase = pickupArmBase;
        armBaseCone = suctionArmBase;
        addRequirements(swerveBase, armBase, armBaseCone);
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
        //System.out.println(getTimeSinceInitialized());
        if (timeSinceInitialized < 100) {
            drive.drive(0.05, 0.0, 0, true);
            armBaseCone.unlock_arm();
        }
        else if (timeSinceInitialized < 2000) {
            armBaseCone.unlock_arm();
            armBase.go_to_position(ArmConstants.DROP_POS);
            drive.drive(0.0, 0.0, 0.0, true);
        }
        else if (timeSinceInitialized < 2300) {
            armBase.go_to_position(ArmConstants.DROP_POS);
            armBaseCone.release_cone();
        }
        else if (timeSinceInitialized < 5000) {
            drive.drive(1.0, 0.1, 0.0, true);
            armBase.go_to_position(ArmConstants.START_POS);//was feeder
        }
        else if (timeSinceInitialized < 6500) {//was 6300
            drive.drive(1.0, 0.0, 0.0, true);
            armBase.go_to_position(ArmConstants.START_POS);//was feeder
        }
        else { // stop
            drive.drive(0.0, 0, 0, true);
            armBase.go_to_position(ArmConstants.START_POS);//was feeder
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
