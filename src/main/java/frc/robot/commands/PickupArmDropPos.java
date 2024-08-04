package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.PickupArmBase;
import frc.robot.subsystems.SuctionArmBase;


public class PickupArmDropPos extends CommandBase {
  /*
   * Pickup Arm Drop Position Command
   * --------------------------------
   * 
   * This command will hold the arm at the drop
   * config position.
   */

  private final PickupArmBase m_armSubsystem;
  private final SuctionArmBase m_suctionSubsystem;
  private final RobotContainer robotContainer;


  public PickupArmDropPos(PickupArmBase armSubsystem, SuctionArmBase suctionSubsystem, RobotContainer container) {
    m_armSubsystem = armSubsystem;
    m_suctionSubsystem = suctionSubsystem;
    robotContainer = container;
    addRequirements(m_armSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_armSubsystem.stop();
  }

  @Override
  public void initialize() {
    m_suctionSubsystem.unlock_arm();
  }

  @Override
  public void execute() {
    m_armSubsystem.go_to_position(ArmConstants.DROP_POS);
  }

  @Override
  public boolean isFinished() {
    //return !robotContainer.getArmDropStatus();
    return true;
  }

}
