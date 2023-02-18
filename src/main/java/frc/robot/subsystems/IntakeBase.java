package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeBase extends SubsystemBase {

  /** Creates a new IntakeBase. */

  private CANSparkMax intakeMotor;
  private RelativeEncoder intakeMotorEncoder;


  public IntakeBase() {
    intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    intakeMotorEncoder = intakeMotor.getEncoder();
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
    double pos = intakeMotorEncoder.getPosition();

    System.out.println("intake motor pos: " + pos);

    if (direction) {
        intakeMotor.set(power);
    }
    else {
        intakeMotor.set(-power);
    }
  }

  public void stop() {
    /*
     * Turn off all motors.
     */

    intakeMotor.set(0.0);
  }
}
