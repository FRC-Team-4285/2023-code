package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeBase;


public class FloorIntakeRelease extends CommandBase {
  /*
   * Floor Intake Release Command
   * ----------------------------
   * 
   * This command will release the floor intake grabber.
   */

   private final IntakeBase m_intakeSubsystem;
 
 
   public FloorIntakeRelease(IntakeBase intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
     addRequirements(m_intakeSubsystem);
   }

  @Override
  public void end(boolean isInterrupted) {
  }

  @Override
  public void initialize() {
    m_intakeSubsystem.release_cone();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
