package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeBase;


public class FloorIntakeGrab extends CommandBase {
  /*
   * Floor Intake Grabber Command
   * ----------------------------
   * 
   * This command will engage the floor intake grabber.
   */

   private final IntakeBase m_intakeSubsystem;
 
 
   public FloorIntakeGrab(IntakeBase intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
     addRequirements(m_intakeSubsystem);
   }

  @Override
  public void end(boolean isInterrupted) {
  }

  @Override
  public void initialize() {
    m_intakeSubsystem.grab_cone();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
