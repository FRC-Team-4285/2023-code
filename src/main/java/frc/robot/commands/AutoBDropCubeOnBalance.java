package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.PickupArmBase;
import frc.robot.subsystems.SuctionArmBase;
import frc.robot.subsystems.SwerveBase;

public class AutoBDropCubeOnBalance extends CommandBase {
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

    public AutoBDropCubeOnBalance(SwerveBase swerveBase, PickupArmBase pickupArmBase, SuctionArmBase suctionArmBase) {
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
        double tilt = drive.getPigeonSensor().getPitch();//degrees?
        double deadzone = 10.0; //degrees
        //drive.drive(0.0, 0.0, 0.0, true);

        if (timeSinceInitialized < 2000) {
            //start of autonomous
            armBaseCone.unlock_arm();
            //take arm out of starting config
            armBase.go_to_position(ArmConstants.DROP_POS);
            //get arm ready to drop cube
        }
        else if (timeSinceInitialized < 2300) {
            armBaseCone.release_cone();
            armBaseCone.release_cube();
            //drop cube
        }
        else if (timeSinceInitialized < 7000) {
            armBase.go_to_position(ArmConstants.FEEDER_POS);
            //put arm back
            drive.drive(1.0, 0.0, 0.0, true);
            //go over balance for out of community points
        }
        else if (timeSinceInitialized < 7500){
            drive.drive(0.0, 0.0, 0.0, true);
        }
        else if (timeSinceInitialized < 9700) {
            drive.drive(-1.0, 0.0, 0.0, true);
            //back up onto balance for auto-balance attempt
        }
        else if (timeSinceInitialized < 12500){
            drive.drive(0.0,0.0,0.0,true);
        }
        else if(timeSinceInitialized < 13000){
            drive.drive(0.0,0.1, 0.0, true);
        }
        else {
            /* Feedback-based balancing code */
            /* use pitch value from pigeon to adjust wheel speed */
            /*if (tilt > deadzone){
                //pitch is too high, decrease pitch
                drive.drive(-0.5, 0.0, 0.0, false);
            }*/
            /*else if (tilt < -deadzone)
            {
                //pitch is too low, increase pitch
                drive.drive(0.5,0.0,0.0, false);

            }*/
            //else{
            drive.drive(0.0, 0.0, 0.0, true);
            //keep robot balanced until end of auto
            //}
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