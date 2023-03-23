package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberArmBase;


public class ClimberUp extends CommandBase {
  /*
   * Climber Up Command
   * ------------------
   * 
   * This command will raise the climber system.
   */

  private final ClimberArmBase m_climberSubsystem;


  public ClimberUp(ClimberArmBase subsystem) {
    m_climberSubsystem = subsystem;
    addRequirements(m_climberSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_climberSubsystem.stop();
  }

  @Override
  public void initialize() {
    m_climberSubsystem.engage_climber(ClimberConstants.CLIMBER_DIRECTION_UP, ClimberConstants.CLIMBER_MOTOR_POWER);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
