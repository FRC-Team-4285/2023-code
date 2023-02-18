package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmBase;


public class ArmDown extends CommandBase {

  // Arm Subsystem
  private final ArmBase m_armSubsystem;


  public ArmDown(ArmBase subsystem) {

    m_armSubsystem = subsystem;

    addRequirements(m_armSubsystem);

  }

  @Override
  public void end(boolean isInterrupted) {
    m_armSubsystem.stop();
  }


  @Override

  public void initialize() {
    m_armSubsystem.engage_arm(ArmConstants.ARM_DIRECTION_DOWN);
  }


  @Override

  public boolean isFinished() {

    return true;

  }

}
