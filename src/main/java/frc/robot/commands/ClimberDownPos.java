package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberArmBase;


public class ClimberDownPos extends CommandBase {
  /*
   * Climber Down Command
   * --------------------
   * 
   * This command will lower the climber system.
   */

  private final ClimberArmBase m_climberSubsystem;


  public ClimberDownPos(ClimberArmBase subsystem) {
    m_climberSubsystem = subsystem;
    addRequirements(m_climberSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    System.out.println("CAN YOU SEE ME? HELLO FROM THE OTHER SIDEEEEEEE.");
    m_climberSubsystem.stop();
  }

  @Override
  public void initialize() {
    m_climberSubsystem.go_to_position(ClimberConstants.CLIMBER_POS_DOWN, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
