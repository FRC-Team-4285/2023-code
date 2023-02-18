package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberBase;


public class ClimberDown extends CommandBase {

  // Climber Subsystem
  private final ClimberBase m_climberSubsystem;


  public ClimberDown(ClimberBase subsystem) {

    m_climberSubsystem = subsystem;

    addRequirements(m_climberSubsystem);

  }

  @Override
  public void end(boolean isInterrupted) {
    m_climberSubsystem.stop();
  }


  @Override

  public void initialize() {
    m_climberSubsystem.engage_climber(ClimberConstants.CLIMBER_DIRECTION_DOWN);
  }


  @Override

  public boolean isFinished() {

    return true;

  }

}