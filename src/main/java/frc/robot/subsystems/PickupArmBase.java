package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PickupArmBase extends SubsystemBase {

  /** Creates a new ArmBase. */

  private CANSparkMax armMotor;
  private SparkMaxPIDController armMotorPID;
  private SparkMaxAbsoluteEncoder armMotorEncoder;
  private boolean arm_direction;
  private RelativeEncoder encoder;
  private double desiredPosition;
  private boolean inPosition;
  private RobotContainer robotContainer;

  public PickupArmBase(RobotContainer container) {
    robotContainer = container;
    armMotor = new CANSparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);

    armMotorEncoder = armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    armMotorPID = armMotor.getPIDController();
    armMotorPID.setFeedbackDevice(armMotorEncoder);

    desiredPosition = 99999; // starting value.
    inPosition = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentPosition = getEncoderValue();
    SmartDashboard.putNumber("Arm Position", currentPosition);
     System.out.println("arm position " + currentPosition);//was commented out
    if (Math.abs(currentPosition - desiredPosition) < 0.25) {
      inPosition = true;
    } else {
      inPosition = false;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getEncoderValue() {
    return armMotorEncoder.getPosition();
  }

  public void engage_arm(boolean direction) {
    /*
     * Engage arm motor.
     */

    arm_direction = direction;

    double power = ArmConstants.ARM_MOTOR_POWER;
    // need to set 50.0 as offset for this encoder
    double pos = getEncoderValue();

    boolean isSafe = getIsSafe(direction, pos);
    if (!isSafe) {
      armMotor.set(0);
      return;
    }

    // System.out.println(power);
    if (direction) {
      armMotor.set(power);
    }
    else {
      armMotor.set(-power);
    }
  }

  private boolean getIsSafe(boolean direction, double pos) {
    return true;
  }

  public void stop() {
    /*
     * Turn off all motors.
     */

    armMotor.set(0.0);
    inPosition = false;
    desiredPosition = 9999;
  }

  public void go_to_position(double position) {
    desiredPosition = position;
    armMotorPID = armMotor.getPIDController();
    armMotorPID.setP(5.0);
    armMotorPID.setI(0.0);
    armMotorPID.setD(0.0);
    armMotorPID.setIZone(0.0);
    armMotorPID.setFF(0.0);
    armMotorPID.setOutputRange(-35.5, 35.5);
    armMotorPID.setReference(position, ControlType.kPosition);
  }

  public boolean getInPosition() {
    return inPosition;
  }

}
