package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeBase;


public class ConeGrabberIngestUp extends CommandBase {
  /*
   * Intake Up Command
   * -----------------
   * 
   * This command will raise the intake system.
   */

  private final IntakeBase m_intakeSubsystem;


  public ConeGrabberIngestUp(IntakeBase subsystem) {
    m_intakeSubsystem = subsystem;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_intakeSubsystem.stop();
  }

  @Override
  public void initialize() {
    m_intakeSubsystem.engage_intake(IntakeConstants.INTAKE_DIRECTION_UP);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
