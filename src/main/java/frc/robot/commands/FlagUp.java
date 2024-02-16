package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FlagConstants;
import frc.robot.subsystems.flagBase;

public class FlagUp extends CommandBase {
  /*
   * Flag Up Command
   * ------------------
   * 
   * This command will raise the Flag. 
   */

  private final flagBase m_flagSubsystem;


  public FlagUp(flagBase subsystem) {
    m_flagSubsystem = subsystem;
    addRequirements(m_flagSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_flagSubsystem.moveFlag(0.0);
  }

  @Override
  public void initialize() {
    m_flagSubsystem.moveFlagToPosition(FlagConstants.FLAG_POS_UP);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
