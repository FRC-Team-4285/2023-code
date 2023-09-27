package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuctionArmBase;


public class Jiggle extends CommandBase {
  /*
   * Jiggle Command
   * --------------
   * 
   * This command will jiggle.
   */

   private final SuctionArmBase m_suctionArmSubsystem;
 
 
   public Jiggle(SuctionArmBase suctionSubsystem) {
     m_suctionArmSubsystem = suctionSubsystem;
     addRequirements(suctionSubsystem);
   }

  @Override
  public void end(boolean isInterrupted) {
    m_suctionArmSubsystem.engage_jiggle(false);
  }

  @Override
  public void initialize() {
    m_suctionArmSubsystem.engage_jiggle(true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
