package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.PickupArmBase;
import frc.robot.subsystems.SuctionArmBase;


public class PickupArmStartPos extends CommandBase {
  /*
   * Pickup Arm Start Position Command
   * ---------------------------------
   * 
   * This command will hold the arm at the starting
   * config position.
   */

  private final PickupArmBase m_armSubsystem;
  private final SuctionArmBase m_suctionSubsystem;
  private double startTime;
  private final RobotContainer robotContainer;


  public PickupArmStartPos(PickupArmBase armSubsystem, SuctionArmBase suctionSubsystem, RobotContainer container) {
    robotContainer = container;
    m_armSubsystem = armSubsystem;
    m_suctionSubsystem = suctionSubsystem;
    addRequirements(m_armSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_armSubsystem.stop();
    double currentEncoderPos = m_armSubsystem.getEncoderValue();
    if (ArmConstants.START_POS > currentEncoderPos) {
      m_suctionSubsystem.lock_arm();
    }
  }

  @Override
  public void initialize() {
    startTime = getCurrentTime();
  }

  @Override
  public void execute() {
    double currentEncoderPos = m_armSubsystem.getEncoderValue();
    double timeSinceInitialized = getTimeSinceInitialized();

    if (ArmConstants.START_POS < currentEncoderPos && timeSinceInitialized < 650) {
      m_armSubsystem.go_to_position(ArmConstants.START_PLUS_POS);
      m_suctionSubsystem.unlock_arm();
    }
    else {
      m_armSubsystem.go_to_position(ArmConstants.START_PLUS_POS);
    }
  }

  @Override
  public boolean isFinished() {
    //return !robotContainer.getArmStartStatus();
    return true;
  }

  private double getCurrentTime() {
    /*
    * Returns current time in milliseconds.
    */

    return System.currentTimeMillis();
  }

  private double getTimeSinceInitialized() {
      return getCurrentTime() - startTime;
  }
}
