package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeBase extends SubsystemBase {

  /** Creates a new IntakeBase. */

  private CANSparkMax intake_motor;
  private RelativeEncoder intake_motor_encoder;

  public IntakeBase() {

    intake_motor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    intake_motor_encoder = intake_motor.getEncoder();

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

  public void engage_intake(boolean direction) {
    /*
     * Engage intake motor.
     */

    double power = IntakeConstants.INTAKE_MOTOR_POWER;
    double pos = intake_motor_encoder.getPosition();

    System.out.println("intake motor pos: " + pos);

    if (direction) {
        intake_motor.set(power);
    }
    else {
        intake_motor.set(-power);
    }
  }

  public void stop() {
    /*
     * Turn off all motors.
     */

     intake_motor.set(0.0);
  }
}