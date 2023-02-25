package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PneumaticChannels;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

// import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeBase extends SubsystemBase {

  /** Creates a new IntakeBase. */

  private CANSparkMax intakeMotor;
  private RelativeEncoder intakeMotorEncoder;
  private DoubleSolenoid intakeExtender;
  private DoubleSolenoid intakeGrabber;


  public IntakeBase() {
    intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    intakeMotorEncoder = intakeMotor.getEncoder();

    intakeExtender = new DoubleSolenoid(
      PneumaticsModuleType.REVPH,
      PneumaticChannels.FLOOR_EXTEND_OFF,
      PneumaticChannels.FLOOR_EXTEND_ON
    );

    intakeGrabber = new DoubleSolenoid(
      PneumaticsModuleType.REVPH,
      PneumaticChannels.FLOOR_GRAB_OFF,
      PneumaticChannels.FLOOR_GRAB_ON
    );
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

    // System.out.println("intake motor pos: " + pos);

    if (direction) {
        intakeMotor.set(power);
    }
    else {
        intakeMotor.set(-power);
    }
  }

  public void extend_intake(){
    intakeExtender.set(Value.kForward);
  }

  public void retract_intake(){
    intakeExtender.set(Value.kReverse);
  }
  
  public void grab_cone(){
    intakeGrabber.set(Value.kForward);
  }

  public void release_cone(){
    intakeGrabber.set(Value.kReverse);
  }

  public void stop() {
    /*
     * Turn off all motors.
     */

    intakeMotor.set(0.0);
  }
}
