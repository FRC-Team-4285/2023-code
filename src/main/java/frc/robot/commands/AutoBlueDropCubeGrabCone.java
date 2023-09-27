package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.PickupArmBase;
import frc.robot.subsystems.SuctionArmBase;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.SwerveBase;

public class AutoBlueDropCubeGrabCone extends CommandBase {
    /* This auto scores a cube and attempts to grab a cone off the floor! */
    private double startTime = 0.0;
    private final SwerveBase drive;
    private final PickupArmBase armBase;
    private final SuctionArmBase armBaseCone;
    private final IntakeBase armBaseIntake;

    public AutoBlueDropCubeGrabCone(SwerveBase swerveBase, PickupArmBase pickupArmBase, SuctionArmBase suctionArmBase, IntakeBase intakeArmBase) {
        drive = swerveBase;
        armBase = pickupArmBase;
        armBaseCone = suctionArmBase;
        armBaseIntake = intakeArmBase;
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
        if (timeSinceInitialized < 100) {
            drive.drive(0.05, 0.0, 0, true);
            armBaseCone.unlock_arm();
        }
        else if (timeSinceInitialized < 2000) {
            armBaseCone.unlock_arm();
            armBase.go_to_position(ArmConstants.DROP_POS);
            drive.drive(0.0, 0.0, 0.0, true);
        }
        else if (timeSinceInitialized < 2500) {
            armBase.go_to_position(ArmConstants.DROP_POS);
            armBaseCone.release_cone();
        }
        else if (timeSinceInitialized < 3500) {
            drive.drive(0.0, 0.2, 0.0, true);
            armBase.go_to_position(ArmConstants.FEEDER_POS);
        }
        else if (timeSinceInitialized < 6500) { //was 6200
            drive.drive(1.5, 0.0, 0.0, true);
            armBase.go_to_position(ArmConstants.FEEDER_POS);
        }
        else if (timeSinceInitialized < 6750) { 
            drive.drive(0.0, 0.0, 0.0, true);
            armBase.go_to_position(ArmConstants.FEEDER_POS);
            armBaseIntake.extend_intake();
            armBaseIntake.grab_cone();
            
        }    
        else if (timeSinceInitialized < 7000) { 
            drive.drive(0.0, 0.0, 0.0, true);
            armBase.go_to_position(ArmConstants.FEEDER_POS);
            armBaseIntake.retract_intake();
            armBaseIntake.engage_intake(true);
        }    
        else if (timeSinceInitialized < 7500) { 
            drive.drive(0.0, 0.0, 0.0, true);
            armBase.go_to_position(ArmConstants.FEEDER_POS);
            armBase.engage_arm(true);
        }    
        else if (timeSinceInitialized < 10700) { 
            drive.drive(1.5, 0.0, 0.0, true);
            armBase.go_to_position(ArmConstants.START_POS);
            armBaseIntake.release_cone();
        }    
        else if (timeSinceInitialized < 13000) { 
            drive.drive(0.0, 0.0, 0.0, true);
            armBase.go_to_position(ArmConstants.DROP_POS);
            armBaseCone.release_cone();
            
        } 
        else if (timeSinceInitialized < 15000) { 
            drive.drive(0.0, 0.0, 0.0, true);
            armBase.go_to_position(ArmConstants.START_POS);
        }   
        
        else { // stop
            drive.drive(0.0, 0, 0, true);
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
