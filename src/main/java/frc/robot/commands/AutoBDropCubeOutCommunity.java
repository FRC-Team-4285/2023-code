package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.PickupArmBase;
import frc.robot.subsystems.SuctionArmBase;
import frc.robot.subsystems.SwerveBase;

public class AutoBDropCubeOutCommunity extends CommandBase {
   private double startTime = 0.0;
   private final SwerveBase drive;
   private final PickupArmBase armBase;
   private final SuctionArmBase armBaseCone;
    public AutoBDropCubeOutCommunity(SwerveBase swerveBase, PickupArmBase pickupArmBase, SuctionArmBase suctionArmBase) {
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
        return System.currentTimeMillis();
    }

    private double getTimeSinceInitialized() {
        return getCurrentTime() - startTime;
    }
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double timeSinceInitialized = getTimeSinceInitialized();
        if (timeSinceInitialized < 1900) {
            //start of autonomous
            armBaseCone.unlock_arm();
            //take arm out of starting config
            armBase.go_to_position(ArmConstants.DROP_POS);
            //get arm ready to drop cube
        }
        else if (timeSinceInitialized < 2100) {
            armBaseCone.release_cone();
            armBaseCone.release_cube();
            //drop cube
        }
        else if (timeSinceInitialized < 7250) {
            armBase.go_to_position(ArmConstants.START_POS);
            //put arm back
            drive.drive(1.0, 0.0, 0.0, true);
            //go over balance for out of community points
        }
        else {
            armBase.go_to_position(ArmConstants.START_POS);
            drive.drive(0.0,0.0,0.0,true);
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