package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuctionArmBase;


public class PickupArmRelease extends CommandBase {
  /*
   * Intake Down Command
   * -------------------
   * 
   * This command will lower the intake system.
   */

   private final SuctionArmBase m_suctionArmSubsystem;
 
 
   public PickupArmRelease(SuctionArmBase suctionSubsystem) {
     m_suctionArmSubsystem = suctionSubsystem;
     addRequirements(m_suctionArmSubsystem);
   }

  @Override
  public void end(boolean isInterrupted) {
  }

  @Override
  public void initialize() {
    m_suctionArmSubsystem.release_cone();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
