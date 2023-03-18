package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuctionArmBase;


public class ConeGrab extends CommandBase {
  /*
   * Cone Grab Command
   * -------------------
   * 
   * This grabs the cone.
   */

  private final SuctionArmBase m_suctionArmSubsystem;


  public ConeGrab(SuctionArmBase subsystem) {
    m_suctionArmSubsystem = subsystem;
    addRequirements(m_suctionArmSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_suctionArmSubsystem.release_cone();
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
