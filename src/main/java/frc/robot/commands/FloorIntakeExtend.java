package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.SuctionArmBase;


public class FloorIntakeExtend extends CommandBase {
  /*
   * Intake Down Command
   * -------------------
   * 
   * This command will lower the intake system.
   */

  private final IntakeBase m_intakeBase;


  public FloorIntakeExtend(IntakeBase subsystem) {
    m_intakeBase = subsystem;
    addRequirements(m_intakeBase);
  }

  @Override
  public void end(boolean isInterrupted) {
    // Do nothing.
  }

  @Override
  public void initialize() {
    m_intakeBase.extend_intake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
