package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.SuctionArmBase;


public class ArmPiston extends CommandBase {
  /*
   * Intake Down Command
   * -------------------
   * 
   * This command will lower the intake system.
   */

   private final SuctionArmBase m_suctionArmSubsystem;
 
 
   public ArmPiston(SuctionArmBase suctionSubsystem) {
     m_suctionArmSubsystem = suctionSubsystem;
     addRequirements(m_suctionArmSubsystem);
   }

  @Override
  public void end(boolean isInterrupted) {
    m_suctionArmSubsystem.unlock_arm();
  }

  @Override
  public void initialize() {
    m_suctionArmSubsystem.lock_arm();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
