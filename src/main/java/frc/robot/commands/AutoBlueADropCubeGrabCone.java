package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.PickupArmBase;
import frc.robot.subsystems.SuctionArmBase;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.SwerveBase;


public class AutoBlueADropCubeGrabCone extends CommandBase {
    /* This auto scores a cube and attempts to grab a cone off the floor! */
    private double startTime = 0.0;
    private final SwerveBase drive;
    private final PickupArmBase armBase;
    private final SuctionArmBase armBaseCone;
    private final IntakeBase armBaseIntake;

    public AutoBlueADropCubeGrabCone(SwerveBase swerveBase, PickupArmBase pickupArmBase, SuctionArmBase suctionArmBase, IntakeBase intakeArmBase) {
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
            drive.drive(0.0, 0.0, 0.0, true);
            armBaseCone.unlock_arm();
            armBaseCone.grab_cone();
            
        }
        else if (timeSinceInitialized < 1000) {
            armBaseCone.unlock_arm();
            armBase.go_to_position(ArmConstants.DROP_POS);
            drive.drive(0.0, 0.0, 0.0, true);
        }
        else if (timeSinceInitialized < 2000) {
            armBase.go_to_position(ArmConstants.DROP_POS);
            
        }
        // else if (timeSinceInitialized < 3000) {
        //     drive.drive(0.0, 0.0, 0.0, true);
        //     armBase.go_to_position(ArmConstants.FEEDER_POS);
        // }
        // else if (timeSinceInitialized < 3500) {
        //     drive.drive(0.0, 0.0, 0.0, true);
        //     armBase.go_to_position(ArmConstants.FEEDER_POS);
        // }
        else if (timeSinceInitialized < 5500) { //was 6200
            armBaseCone.release_cone();
            drive.drive(1.0, 0.0, 0.05, true);
            armBase.go_to_position(ArmConstants.START_POS);
        
        }
        else if (timeSinceInitialized < 7900) { //was 6200
            armBaseCone.release_cone();
            drive.drive(1.0, 0.0, 0.00, true);
            armBase.go_to_position(ArmConstants.START_POS);
           // armBaseIntake.extend_intake();
           // armBaseIntake.go_to_position(IntakeConstants.INTAKE_EXTEND_POS);
            
        }
        // else if (timeSinceInitialized < 8250) { //was8250
        //     drive.drive(0.0, 0.0, 0.0, true);
        //     armBase.go_to_position(ArmConstants.FEEDER_POS);
        //     armBaseIntake.grab_cone();
        // }    
        // else if (timeSinceInitialized < 8650) { //was 8650
        //      drive.drive(0.0, 0.0, 0.0, true);
        //      armBase.go_to_position(ArmConstants.FEEDER_POS);
        //      armBaseIntake.retract_intake();
        //      armBaseIntake.go_to_position(IntakeConstants.INTAKE_RETRACT_POS);
             
             
        //     // armBaseIntake.go_to_position(IntakeConstants.INTAKE_RETRACT_POS);
        //      //armBaseIntake.go_to_position();
        // }    
        // else if (timeSinceInitialized < 8850) { //was 8850
        //      drive.drive(0.0, 0.0, 0.0, true);
        //      armBase.go_to_position(ArmConstants.FEEDER_POS);
        //      armBaseCone.grab_cone();
        //      armBaseIntake.release_cone();
        //     // armBase.engage_arm(true);
        // }  
        
        // else if (timeSinceInitialized < 13000) { 
        //     drive.drive(-1.0, 0.09, 0.0, true);
        //     armBase.go_to_position(ArmConstants.START_POS);
        // }      
       
    //     else if (timeSinceInitialized < 14250) { 
    //         drive.drive(-1.0, 0.0, 0.0, true);
    //         armBase.go_to_position(ArmConstants.DROP_POS);
    //     }    
    //     // else if (timeSinceInitialized < 13500) { 
    //     //     drive.drive(0.0, 0.0, 0.0, true);
    //     //     // armBase.go_to_position(ArmConstants.DROP_POS);
    //     //     // armBaseCone.release_cone();
            
    //     //} 
    //     else if (timeSinceInitialized < 14999) { 
    //          drive.drive(1.0, 0.0, 0.0, true);
    //          armBase.go_to_position(ArmConstants.DROP_POS);
    //          armBaseIntake.release_cone();
    //     }   
    //     else if (timeSinceInitialized < 15000) { 
    //         drive.drive(0.0, 0.0, 0.0, true);
    //         armBase.go_to_position(ArmConstants.START_POS);
    //         armBaseCone.release_cone();
    //    }  
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
