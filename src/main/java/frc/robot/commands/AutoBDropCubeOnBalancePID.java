package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.PickupArmBase;
import frc.robot.subsystems.SuctionArmBase;
import frc.robot.subsystems.SwerveBase;

public class AutoBDropCubeOnBalancePID extends CommandBase {

   private double startTime = 0.0;
   private double timeOfBalanceAttempt = 0.0;
   private int waitForBalance = 500; //milliseconds, was 667
   private int nudgeTime = 400; //milliseconds
   private double nudgePower = 0.65;//was 0.67
   private Boolean AttemptingBalancePos = false;
   private Boolean AttemptingBalanceNeg = false;
   private Boolean isClimbingBalance = false;
   private double timeOfClimbing = 0.0;
   private Boolean ClimbedOnBalance = false;
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
        double timeSinceClimbing = timeSinceInitialized - timeOfClimbing;
        double tilt = drive.getPigeonSensor().getPitch();//degrees
        double climbThreshold = 10.0;//degrees
        double deadzone = 2.5; //degrees

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
        else if (timeSinceInitialized < 7750){
            armBase.go_to_position(ArmConstants.START_POS);
            drive.drive(0.0, 0.0, 0.0, true);
        }
        /*
        else if (timeSinceInitialized < 10250) {
            armBase.go_to_position(ArmConstants.START_POS);
            drive.drive(-1.0, 0.0, 0.0, true);
            //back up onto balance for auto-balance attempt
        }
        else if (timeSinceInitialized < 10500){
            armBase.go_to_position(ArmConstants.START_POS);
            drive.drive(0.0,0.0,0.0,true);
        }
        */
        //*
        else if(!ClimbedOnBalance){
            armBase.go_to_position(ArmConstants.START_POS);
            drive.drive(-1.0, 0.0, 0.0, true);
            if(Math.abs(tilt) < climbThreshold){
                //tilt went below climbThreshold, reset timer
                timeOfClimbing = timeSinceInitialized;
                isClimbingBalance = false;
            }

            else if((Math.abs(tilt) > climbThreshold) && !(isClimbingBalance)){
                //tilt is more than climbThreshold, start timing climb
                System.out.println("Tilt Detected!");
                timeOfClimbing = timeSinceInitialized;
                isClimbingBalance = true;
            }
            
            if(timeSinceClimbing > 100){
                //tilt has been above 10 degrees for more than 100 milliseconds
                //assume that robot has sucessfully climbed onto balance
                System.out.println("Tilt Confirmed, starting climb!");
                ClimbedOnBalance = true;
                timeOfClimbing = timeSinceInitialized;
            }
        }
        else if(timeSinceClimbing < 3000){
            armBase.go_to_position(ArmConstants.START_POS);
            drive.drive(-1.0, 0.0, 0.0, true);
        }
        //*/
        else {
            armBase.go_to_position(ArmConstants.START_POS);
            //Nudge robot if balance isn't level
            if((tilt > deadzone) && (timeSinceBalanceAttempt > waitForBalance) && !(AttemptingBalanceNeg)){
                //above condition requires robot to be tilted for a certain amount of time since the last nudge
                System.out.println("Starting Negative Nudge");
                /*set flag for negative nudge */
                AttemptingBalanceNeg = true;
            }
            if((tilt < -deadzone) && (timeSinceBalanceAttempt > waitForBalance) && !(AttemptingBalancePos)){
                System.out.println("Starting Positive Nudge");
                /*set flag for positive nudge */
                AttemptingBalancePos = true;
            }
            if (AttemptingBalanceNeg){
                if(timeSinceBalanceAttempt < (waitForBalance + nudgeTime)){
                    drive.drive(-nudgePower, 0.0, 0.0, true);
                }
                else{
                    System.out.println("Completed Negative Nudge");
                    //Balance attempt complete, reset timer since last balance
                    timeOfBalanceAttempt = timeSinceInitialized;
                    /*reset flag for negative nudge */
                    AttemptingBalanceNeg = false;
                }

            }
            else if(AttemptingBalancePos){
                if(timeSinceBalanceAttempt < waitForBalance + nudgeTime){
                    drive.drive(nudgePower, 0.0, 0.0, true);
                }
                else{
                    System.out.println("Completed Positive Nudge");
                    timeOfBalanceAttempt = timeSinceInitialized;
                    /*reset flag for positive nudge */
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