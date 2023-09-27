package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.SuctionArmBase;


public class ConeGrabberIngestCone extends CommandBase {
  /*
   * Cone Grabber Rotate Up
   * -------------------
   * 
   * This command will lower the intake system.
   */

   private final IntakeBase m_intakeSubsystem;
 
 
   public ConeGrabberIngestCone(IntakeBase intakeSubsystem) {
     m_intakeSubsystem = intakeSubsystem;
     addRequirements(m_intakeSubsystem);
   }

  @Override
  public void end(boolean isInterrupted) {
    m_intakeSubsystem.stop();
  }

  @Override
  public void initialize() {
    m_intakeSubsystem.go_to_position(-60.453);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
