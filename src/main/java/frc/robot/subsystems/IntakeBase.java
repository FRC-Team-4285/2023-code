package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.Constants.HardwareCAN;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PneumaticChannels;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeBase extends SubsystemBase {
  /*Creates a new IntakeBase*/
  private CANSparkMax intakeMotor;
  private SparkMaxPIDController intakeMotorPID;
  private DoubleSolenoid intakeExtender;
  private DoubleSolenoid intakeGrabber;

  public IntakeBase(RobotContainer container) {
    intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    intakeExtender = new DoubleSolenoid(
      HardwareCAN.PNEUMATIC_HUB,
      PneumaticsModuleType.REVPH,
      PneumaticChannels.FLOOR_EXTEND_ON,
      PneumaticChannels.FLOOR_EXTEND_OFF
    );

    intakeGrabber = new DoubleSolenoid(
      HardwareCAN.PNEUMATIC_HUB,
      PneumaticsModuleType.REVPH,
      PneumaticChannels.FLOOR_GRAB_ON,
      PneumaticChannels.FLOOR_GRAB_OFF
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
    /*Engage intake motor*/
    double power = IntakeConstants.INTAKE_MOTOR_POWER;

    if (direction) {
        intakeMotor.set(power);
    }
    else {
        intakeMotor.set(-power);
    }
  }

  public void go_to_position(double position) {
    intakeMotorPID = intakeMotor.getPIDController();
    intakeMotorPID.setP(0.3);
    intakeMotorPID.setI(0.0);
    intakeMotorPID.setD(0.0);
    intakeMotorPID.setIZone(0.0);
    intakeMotorPID.setFF(0.0);
    intakeMotorPID.setOutputRange(-0.7, 0.7);
    intakeMotorPID.setReference(position, ControlType.kPosition);
  }

  public void extend_intake() {
    intakeExtender.set(Value.kForward);
  }

  public void retract_intake() {
    intakeExtender.set(Value.kReverse);
  }

  public void grab_cone() {
    intakeGrabber.set(Value.kReverse);
  }

  public void release_cone() {
    intakeGrabber.set(Value.kForward);
  }

  public void stop() {
    /*
     * Turn off all motors.
     */

    intakeMotor.set(0.0);
  }
}
