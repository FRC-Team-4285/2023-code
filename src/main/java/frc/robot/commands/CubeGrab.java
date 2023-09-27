package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuctionArmBase;


public class CubeGrab extends CommandBase {
  /*
   * Intake Down Command
   * -------------------
   * 
   * This command will lower the intake system.
   */

  private final SuctionArmBase m_suctionArmSubsystem;


  public CubeGrab(SuctionArmBase subsystem) {
    m_suctionArmSubsystem = subsystem;
    addRequirements(m_suctionArmSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_suctionArmSubsystem.release_cube();
  }

  @Override
  public void initialize() {
    m_suctionArmSubsystem.grab_cube();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
