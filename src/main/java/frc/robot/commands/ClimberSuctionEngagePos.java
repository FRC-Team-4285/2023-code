package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.SuctionConstants;
import frc.robot.subsystems.ClimberArmBase;
import frc.robot.subsystems.SuctionCupBase;


public class ClimberSuctionEngagePos extends CommandBase {
  /*
   * Climber Down Command
   * --------------------
   * 
   * This command will lower the climber system.
   */

  private final ClimberArmBase m_climberSubsystem;
  private final SuctionCupBase m_suctionSubsystem;
  private int commandState = 0;
  private double startTime = 0;


  public ClimberSuctionEngagePos(ClimberArmBase climberSubsystem, SuctionCupBase suctionSubsystem) {
    m_climberSubsystem = climberSubsystem;
    m_suctionSubsystem = suctionSubsystem;
    addRequirements(m_climberSubsystem);
  }

  private double getCurrentTime() {
    /*
     * Returns current time in milliseconds.
     */

    return System.currentTimeMillis();
  }

  public void periodic() {
    m_climberSubsystem.go_to_position(ClimberConstants.CLIMBER_POS_DOWN_SUCTION, false);

    if (commandState == 0 && getCurrentTime() - startTime > 200) {
      // give a few moments for the arm to go down.
      commandState = 1;
    }
    else if (commandState == 1) {
      // engage suction
      commandState = 2;
      m_suctionSubsystem.engageSuctionCup(SuctionConstants.SUCTION_CUP_ENGAGE);
    }
    else if (commandState == 2) {
      // periodically check if we're complete
      if (m_suctionSubsystem.checkIsPumpReady() == SuctionConstants.PUMP_COMPLETE) {
        // once we're complete set our state to 3.
        commandState = 3;
      }
    }
  }

  @Override
  public void end(boolean isInterrupted) {
    m_climberSubsystem.stop();
    m_suctionSubsystem.stop();
  }

  @Override
  public void initialize() {
    startTime = getCurrentTime();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

}
