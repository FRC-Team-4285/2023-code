package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.PickupArmBase;
import frc.robot.subsystems.SuctionArmBase;
import frc.robot.subsystems.SwerveBase;

public class AutoBDropCubeOnBalancePID extends CommandBase {

   private double startTime = 0.0;
   private double timeOfBalanceAttempt = 0.0;
   private int waitForBalance = 750; //milliseconds
   private int nudgeTime = 400; //milliseconds
   private Boolean AttemptingBalancePos = false;
   private Boolean AttemptingBalanceNeg = false;
   private final SwerveBase drive;
   private final PickupArmBase armBase;
   private final SuctionArmBase armBaseCone;

    public AutoBDropCubeOnBalancePID(SwerveBase swerveBase, PickupArmBase pickupArmBase, SuctionArmBase suctionArmBase) {
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
        double timeSinceBalanceAttempt = timeSinceInitialized - timeOfBalanceAttempt;
        double tilt = drive.getPigeonSensor().getPitch();//degrees
        double deadzone = 5.0; //degrees

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
        else if (timeSinceInitialized < 9800) {
            drive.drive(-1.0, 0.0, 0.0, true);
            //back up onto balance for auto-balance attempt
        }
        else if (timeSinceInitialized < 11000){
            drive.drive(0.0,0.0,0.0,true);
        }
        /*
        else if(timeSinceInitialized < 13000){
            //drive.drive(0.0,0.1, 0.0, true);
        }
        */
        else {
            //Nudge robot if balance isn't level
            if((tilt > deadzone) && (timeSinceBalanceAttempt > waitForBalance) && !(AttemptingBalanceNeg)){
                System.out.println("Starting Negative Nudge");
                AttemptingBalanceNeg = true;
            }
            if((tilt < -deadzone) && (timeSinceBalanceAttempt > waitForBalance) && !(AttemptingBalancePos)){
                System.out.println("Starting Positive Nudge");
                AttemptingBalancePos = true;
            }
            if (AttemptingBalanceNeg){
                if(timeSinceBalanceAttempt < (waitForBalance + nudgeTime)){
                    drive.drive(-0.5, 0.0, 0.0, true);
                }
                else{
                    System.out.println("Completed Negative Nudge");
                    timeOfBalanceAttempt = timeSinceInitialized;
                    AttemptingBalanceNeg = false;
                }

            }
            else if(AttemptingBalancePos){
                if(timeSinceBalanceAttempt < waitForBalance + nudgeTime){
                    drive.drive(0.5, 0.0, 0.0, true);
                }
                else{
                    System.out.println("Completed Positive Nudge");
                    timeOfBalanceAttempt = timeSinceInitialized;
                    AttemptingBalancePos = false;
                }
            }
            else{
                drive.drive(0.0,0.0,0.0,true);
            }

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