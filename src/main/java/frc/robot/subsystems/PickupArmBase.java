package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAlternateEncoder;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PickupArmBase extends SubsystemBase {

  /** Creates a new ArmBase. */

  private CANSparkMax armMotor;
  private SparkMaxPIDController armMotorPID;
  private AbsoluteEncoder armMotorEncoder;
  private RelativeEncoder armMotorRelEncoder;
  private boolean arm_direction;
  private RelativeEncoder encoder;
  private double desiredPosition;
  private boolean inPosition;
  private RobotContainer robotContainer;

  //pickup arm has an encoder connected to SPARK MAX
  //said encoder is capable of operating in duty-cycle mode
  //How do you specify the encoder mode to the SPARK MAX???


  public PickupArmBase(RobotContainer container) {
    robotContainer = container;
    armMotor = new CANSparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);

    // armMotorEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);
    // armMotorEncoder.setPositionConversionFactor(0.1);
    armMotorRelEncoder = armMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);

    armMotorPID = armMotor.getPIDController();
    armMotorPID.setFeedbackDevice(armMotorRelEncoder);

    desiredPosition = 99999; // starting value.
    inPosition = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentPosition = getEncoderValue();
    SmartDashboard.putNumber("Arm Position", currentPosition);
     //System.out.println("arm position " + currentPosition);
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
    return armMotorRelEncoder.getPosition();
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
    
    // boolean isSafe = (pos > 0 && pos < 230);
    // if (!isSafe) {
    //   // We know that when within this block we are already out of bounds.
    //   // So we only need to know about one side to know whether or not we exceeded
    //   // the other side. If we are not below 0, we must be above 100; if we are not above 100,
    //   // we must be below 0.
    //   boolean bound_dir_exceeded = (pos < 0) ? true : false;
  
    //   // bound_dir_exceeded = TRUE when we are BELOW 0
    //   // bound_dir_exceeded = FALSE when we are ABOVE 100
    //   // direction = TRUE when we are INCREASING/RAISING ARM
    //   // direction = FALSE when we are DECREASING/LOWERING ARM
  
    //   if (direction && !bound_dir_exceeded) {
    //     // if we are INCEASING and we are ABOVE 100, this is NOT OK.
    //     return false;
    //   }
    //   else if (!direction && bound_dir_exceeded) {
    //     // if we are DECREASING and we are BELOW 0, this is NOT OK.
    //     return false;
    //   }
    // }

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
