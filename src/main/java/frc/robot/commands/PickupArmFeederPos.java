package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
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


  public PickupArmFeederPos(PickupArmBase armSubsystem, SuctionArmBase suctionSubsystem) {
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
    // m_suctionSubsystem.unlock_arm();
  }

  @Override
  public void execute() {
    m_armSubsystem.go_to_position(ArmConstants.FEEDER_POS);
  }

  @Override
  public boolean isFinished() {
    return m_armSubsystem.getInPosition();
  }

}
