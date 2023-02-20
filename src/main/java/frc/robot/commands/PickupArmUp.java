package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.PickupArmBase;


public class PickupArmUp extends CommandBase {
  /*
   * Pickup Arm Up Command
   * ---------------------
   * 
   * This command will raise the pickup arm.
   */

  private final PickupArmBase m_armSubsystem;


  public PickupArmUp(PickupArmBase subsystem) {
    m_armSubsystem = subsystem;
    addRequirements(m_armSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_armSubsystem.stop();
  }

  @Override
  public void initialize() {
    m_armSubsystem.engage_arm(ArmConstants.ARM_DIRECTION_UP);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
