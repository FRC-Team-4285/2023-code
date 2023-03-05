package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.SuctionArmBase;


public class ConeGrab extends CommandBase {
  /*
   * Intake Down Command
   * -------------------
   * 
   * This command will lower the intake system.
   */

   private final SuctionArmBase m_suctionArmSubsystem;
   private final IntakeBase m_intakeSubsystem;
 
 
   public ConeGrab(SuctionArmBase suctionSubsystem, IntakeBase intakeSubsystem) {
     m_suctionArmSubsystem = suctionSubsystem;
     m_intakeSubsystem = intakeSubsystem;
     addRequirements(m_suctionArmSubsystem, m_intakeSubsystem);
   }

  @Override
  public void end(boolean isInterrupted) {
    m_suctionArmSubsystem.release_cone();
    m_intakeSubsystem.release_cone();
  }

  @Override
  public void initialize() {
    m_suctionArmSubsystem.grab_cone();
    m_intakeSubsystem.grab_cone();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
