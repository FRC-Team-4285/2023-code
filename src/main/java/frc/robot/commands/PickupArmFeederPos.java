package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.PickupArmBase;
import frc.robot.subsystems.SuctionArmBase;


public class PickupArmFeederPos extends CommandBase {
  /*
   * Pickup Arm Feeder Position Command
   * ----------------------------------
   * 
   * This command will hold the arm at the feeder
   * config position.
   */

  private final PickupArmBase m_armSubsystem;
  private final SuctionArmBase m_suctionSubsystem;
  private double startTime;
  private final RobotContainer robotContainer;


  public PickupArmFeederPos(PickupArmBase armSubsystem, SuctionArmBase suctionSubsystem, RobotContainer container) {
    robotContainer = container;
    m_armSubsystem = armSubsystem;
    m_suctionSubsystem = suctionSubsystem;
    addRequirements(m_armSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_armSubsystem.stop();
  }

  @Override
  public void initialize() {
    m_suctionSubsystem.unlock_arm();
    startTime = getCurrentTime();
  }

  @Override
  public void execute() {
    double currentEncoderPos = m_armSubsystem.getEncoderValue();
    double timeSinceInitialized = getTimeSinceInitialized();

    if (ArmConstants.START_PLUS_POS < currentEncoderPos && timeSinceInitialized < 500) {
      //System.out.println(currentEncoderPos);
      //System.out.println(timeSinceInitialized);
      m_armSubsystem.go_to_position(ArmConstants.START_PLUS_POS);
      m_suctionSubsystem.unlock_arm();
    }
    else {
      m_armSubsystem.go_to_position(ArmConstants.FEEDER_POS);
    }

  }

  @Override
  public boolean isFinished() {
    //return !robotContainer.getArmFeedStatus();
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
