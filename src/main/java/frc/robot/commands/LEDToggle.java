package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDBase;


public class LEDToggle extends CommandBase {
  /*
   * Suction Cup Release Command
   * ---------------------------
   * 
   * This command disable the suction system, thus
   * releasing the suction cup.
   */

  private final LEDBase m_ledSubsystem;

  public LEDToggle(LEDBase subsystem) {
    m_ledSubsystem = subsystem;
    addRequirements(m_ledSubsystem);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_ledSubsystem.stop();
  }

  @Override
  public void initialize() {
    m_ledSubsystem.toggle_led_strip();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

}
