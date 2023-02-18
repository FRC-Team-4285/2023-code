package frc.robot.subsystems;

import frc.robot.Constants.ClimberConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClimberBase extends SubsystemBase {

  /** Creates a new ClimberBase. */

  private CANSparkMax climber_motor_left;
  private CANSparkMax climber_motor_right;
  private RelativeEncoder climber_motor_left_encoder;
  private RelativeEncoder climber_motor_right_encoder;

  public ClimberBase() {

    climber_motor_left = new CANSparkMax(ClimberConstants.CLIMBER_MOTOR_LEFT_ID, MotorType.kBrushless);
    climber_motor_left_encoder = climber_motor_left.getEncoder();

    climber_motor_right = new CANSparkMax(ClimberConstants.CLIMBER_MOTOR_RIGHT_ID, MotorType.kBrushless);
    climber_motor_right_encoder = climber_motor_right.getEncoder();

  }


  /**

   * Example command factory method.

   *

   * @return a command

   */

  public CommandBase exampleMethodCommand() {

    // Inline construction of command goes here.

    // Subsystem::RunOnce implicitly requires `this` subsystem.

    return runOnce(

        () -> {

          /* one-time action goes here */

        });

  }


  /**

   * An example method querying a boolean state of the subsystem (for example, a digital sensor).

   *

   * @return value of some boolean subsystem state, such as a digital sensor.

   */

  public boolean exampleCondition() {

    // Query some boolean state, such as a digital sensor.

    return false;

  }


  @Override

  public void periodic() {

    // This method will be called once per scheduler run

  }


  @Override

  public void simulationPeriodic() {

    // This method will be called once per scheduler run during simulation

  }

  public void engage_climber(boolean direction) {
    /*
     * Engage throwing motor and determine power based on visual
     * sensor information.
     */

    double power = ClimberConstants.CLIMBER_MOTOR_POWER;
    double pos_left = climber_motor_left_encoder.getPosition();
    double pos_right = climber_motor_right_encoder.getPosition();

    System.out.println("climb motor pos:\n    left: " + pos_left + "\n    right: " + pos_right + "\n\n");

    if (direction) {
        climber_motor_left.set(power);
        climber_motor_right.set(-power);
    }
    else {
        climber_motor_left.set(-power);
        climber_motor_right.set(power);
    }
  }

  public void stop() {
    /*
     * Turn off all motors.
     */

    climber_motor_left.set(0.0);
    climber_motor_right.set(0.0);
  }
}