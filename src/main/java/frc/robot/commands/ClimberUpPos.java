package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberArmBase;


public class ClimberUpPos extends CommandBase {
  /*
   * Climber Up Command
   * ------------------
   * 
   * This command will raise the climber system.
   */

  private final ClimberArmBase m_climberSubsystem;


  public ClimberUpPos(ClimberArmBase subsystem) {
    m_climberSubsystem = subsystem;
    addRequirements(m_climberSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_climberSubsystem.stop();
  }

  @Override
  public void initialize() {
    m_climberSubsystem.go_to_position(ClimberConstants.CLIMBER_POS_UP);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
