package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuctionConstants;
import frc.robot.subsystems.SuctionCupBase;


public class SuctionCupRelease extends CommandBase {
  /*
   * Suction Cup Release Command
   * ---------------------------
   * 
   * This command disable the suction system, thus
   * releasing the suction cup.
   */

  private final SuctionCupBase m_suctionSubsystem;


  public SuctionCupRelease(SuctionCupBase subsystem) {
    m_suctionSubsystem = subsystem;
    addRequirements(m_suctionSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_suctionSubsystem.stop();
  }

  @Override
  public void initialize() {
    m_suctionSubsystem.engageSuctionCup(SuctionConstants.SUCTION_CUP_RELEASE);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

}
