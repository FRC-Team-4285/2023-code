package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class EnterStartingConfig extends CommandBase {
  /*
   * Climber Up Command
   * ------------------
   * 
   * This command will raise the climber system.
   */

  private final RobotContainer robotContainer;


  public EnterStartingConfig(RobotContainer container) {
    robotContainer = container;
   /*  addRequirements(
      robotContainer.climberArmBase
    ); */
  }

  @Override
  public void end(boolean isInterrupted) {
    //robotContainer.climberArmBase.stop();
  }

  @Override
  public void initialize() {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
