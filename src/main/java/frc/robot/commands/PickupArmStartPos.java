package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
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


  public PickupArmStartPos(PickupArmBase armSubsystem, SuctionArmBase suctionSubsystem) {
    m_armSubsystem = armSubsystem;
    m_suctionSubsystem = suctionSubsystem;
    addRequirements(m_armSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_armSubsystem.stop();
    // m_suctionSubsystem.lock_arm();
  }

  @Override
  public void initialize() {
    // m_suctionSubsystem.unlock_arm();
  }

  @Override
  public void execute() {
    m_armSubsystem.go_to_position(ArmConstants.START_POS);
  }

  @Override
  public boolean isFinished() {
    return m_armSubsystem.getInPosition();
  }

}
