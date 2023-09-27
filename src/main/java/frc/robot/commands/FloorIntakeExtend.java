package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.SuctionArmBase;


public class FloorIntakeExtend extends CommandBase {
  /*
   * Intake Down Command
   * -------------------
   * 
   * This command will lower the intake system.
   */

  private final IntakeBase m_intakeSubsystem;


  public FloorIntakeExtend(IntakeBase subsystem) {
    m_intakeSubsystem = subsystem;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    // Do nothing.
  }

  @Override
  public void initialize() {
    m_intakeSubsystem.extend_intake();
    m_intakeSubsystem.go_to_position(IntakeConstants.INTAKE_EXTEND_POS);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
