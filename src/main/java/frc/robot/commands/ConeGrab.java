package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PickupArmBase;


public class ConeGrab extends CommandBase {
  /*
   * Intake Down Command
   * -------------------
   * 
   * This command will lower the intake system.
   */

  private final PickupArmBase m_armSubsystem;


  public ConeGrab(PickupArmBase subsystem) {
    m_armSubsystem = subsystem;
    addRequirements(m_armSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_armSubsystem.release_cone();
  }

  @Override
  public void initialize() {
    m_armSubsystem.grab_cone();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
