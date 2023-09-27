package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberArmBase;


public class ClimberDown extends CommandBase {
  /*
   * Climber Down Command
   * --------------------
   * 
   * This command will raise the climber system.
   */

  private final ClimberArmBase m_climberSubsystem;


  public ClimberDown(ClimberArmBase subsystem) {
    m_climberSubsystem = subsystem;
    addRequirements(m_climberSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_climberSubsystem.stop();
  }

  @Override
  public void initialize() {
    m_climberSubsystem.engage_climber(ClimberConstants.CLIMBER_DIRECTION_DOWN, ClimberConstants.CLIMBER_MOTOR_POWER);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
