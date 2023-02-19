package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuctionConstants;
import frc.robot.subsystems.SuctionCupBase;


public class SuctionCupEngage extends CommandBase {
  /*
   * Suction Cup Engage Command
   * --------------------------
   * 
   * This command enable the suction system.
   */

  private final SuctionCupBase m_suctionSubsystem;


  public SuctionCupEngage(SuctionCupBase subsystem) {
    m_suctionSubsystem = subsystem;
    addRequirements(m_suctionSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_suctionSubsystem.stop();
  }

  @Override
  public void initialize() {
    m_suctionSubsystem.engageSuctionCup(SuctionConstants.SUCTION_CUP_ENGAGE);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

}
