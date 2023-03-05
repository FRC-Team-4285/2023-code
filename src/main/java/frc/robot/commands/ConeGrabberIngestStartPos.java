package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.SuctionArmBase;


public class ConeGrabberIngestStartPos extends CommandBase {
  /*
   * Cone Grabber Ingest Start Pos
   * -----------------------------
   * 
   * This command will set cone grabber ingest to start position.
   */

   private final IntakeBase m_intakeSubsystem;
 
 
   public ConeGrabberIngestStartPos(IntakeBase intakeSubsystem) {
     m_intakeSubsystem = intakeSubsystem;
     addRequirements(m_intakeSubsystem);
   }

  @Override
  public void end(boolean isInterrupted) {
    m_intakeSubsystem.stop();
  }

  @Override
  public void initialize() {
    m_intakeSubsystem.go_to_position(-1.547);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
