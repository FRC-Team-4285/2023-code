package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuctionArmBase;


public class CubeGrabLight extends CommandBase {
  /*
   * Cube Grab Light Command
   * -------------------
   * 
   * This command will make the cube grab light work.
   */

  private final SuctionArmBase m_suctionArmSubsystem;


  public CubeGrabLight(SuctionArmBase subsystem) {
    m_suctionArmSubsystem = subsystem;
    addRequirements(m_suctionArmSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_suctionArmSubsystem.toggle_cube_light(true);
  }

  @Override
  public void initialize() {
    m_suctionArmSubsystem.toggle_cube_light(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
