package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuctionArmBase;


public class PickupArmGrab extends CommandBase {
  /*
   * Pickup Arm Grab Command
   * -----------------------
   * 
   * This command will engage the grabber on the
   * pickup arm.
   */

   private final SuctionArmBase m_suctionArmSubsystem;
 
 
   public PickupArmGrab(SuctionArmBase suctionSubsystem) {
     m_suctionArmSubsystem = suctionSubsystem;
     addRequirements(m_suctionArmSubsystem);
   }

  @Override
  public void end(boolean isInterrupted) {
  }

  @Override
  public void initialize() {
    m_suctionArmSubsystem.grab_cone();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
