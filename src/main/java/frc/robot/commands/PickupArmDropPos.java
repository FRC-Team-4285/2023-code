package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.PickupArmBase;


public class PickupArmDropPos extends CommandBase {
  /*
   * Pickup Arm Down Command
   * -----------------------
   * 
   * This command will lower the pickup arm.
   */

  private final PickupArmBase m_armSubsystem;


  public PickupArmDropPos(PickupArmBase subsystem) {
    m_armSubsystem = subsystem;
    addRequirements(m_armSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_armSubsystem.stop();
  }

  @Override
  public void initialize() {
    m_armSubsystem.go_to_position(ArmConstants.DROP_POS);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
