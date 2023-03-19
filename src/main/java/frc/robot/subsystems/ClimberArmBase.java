package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.HardwareCAN;
import frc.robot.Constants.PneumaticChannels;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

// import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClimberArmBase extends SubsystemBase {

  /** Creates a new ClimberBase. */

  private CANSparkMax climberMotorLeft;
  private CANSparkMax climberMotorRight;
  private RelativeEncoder climberMotorEncoderLeft;
  private RelativeEncoder climberMotorEncoderRight;
  private SparkMaxPIDController climberMotorRightPID;
  private SparkMaxPIDController climberMotorLeftPID;
  private Solenoid climberLiftSolenoid;
  private boolean climber_direction;
  private RobotContainer robotContainer;

  public ClimberArmBase(RobotContainer container) {
    robotContainer = container;
    
    climberMotorLeft = new CANSparkMax(ClimberConstants.CLIMBER_MOTOR_LEFT_ID, MotorType.kBrushless);

    climberMotorRight = new CANSparkMax(ClimberConstants.CLIMBER_MOTOR_RIGHT_ID, MotorType.kBrushless);
    climberMotorRight.setInverted(true);

    climberMotorEncoderLeft = climberMotorLeft.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
    climberMotorEncoderRight = climberMotorRight.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);

    climberMotorLeftPID = climberMotorLeft.getPIDController();
    climberMotorLeftPID.setFeedbackDevice(climberMotorEncoderLeft);
    climberMotorRightPID = climberMotorRight.getPIDController();
    climberMotorRightPID.setFeedbackDevice(climberMotorEncoderRight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double posLeft = getEncoderValueLeft();
    double posRight = getEncoderValueRight();
    // System.out.println("climb motor pos: " + pos);

    boolean isSafe = getIsSafe(climber_direction, posLeft, posRight);
    if (!isSafe) {
      stop();
      return;
    }

  }

  private boolean getIsSafe(boolean direction, double posLeft, double posRight) {
    // // System.out.println(direction + " " + pos);
    // boolean isSafe = (pos > 0 && pos < 101);
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
  
    //   if (direction && bound_dir_exceeded) {
    //     // if we are INCEASING and we are ABOVE 100, this is NOT OK.
    //     return false;
    //   }
    //   else if (!direction && !bound_dir_exceeded) {
    //     // if we are DECREASING and we are BELOW 0, this is NOT OK.
    //     return false;
    //   }
    // }

    return true;
  }

  public double getEncoderValueLeft() {
    return climberMotorEncoderLeft.getPosition();
  }

  public double getEncoderValueRight() {
    return climberMotorEncoderRight.getPosition();
  }

  @Override
  public void simulationPeriodic() {
    // UNUSED
    // This method will be called once per scheduler run during simulation
  }

  public void engage_climber(boolean direction) {
    /*
     * Engage climber motors.
     */

    double power = ClimberConstants.CLIMBER_MOTOR_POWER;
    climber_direction = direction;

    // double pos = getEncoderValue();
    // System.out.println("climb motor pos: " + pos);

    // ----------------------
    // !!! VERY IMPORTANT !!!
    // ----------------------
    // Each motor's power MUST be the same as of each other.
    // If you don't, you will cause a large mechanical error
    // requiring an untimely and difficult manual fix.
    // 
    // More literally:
    //   - If left is + power, right must be + power;
    //   - If left is - power, right must be - power.
    //
    // You have been warned. If you break it, YOU fix it!
    // Yes, that means YOU.

    if (direction) {
      climberLiftSolenoid.set(true);
      climberMotorLeft.set(power);
      climberMotorRight.set(power);
    }
    else {
      climberLiftSolenoid.set(false);
      climberMotorLeft.set(-power);
      climberMotorRight.set(-power);
    }
  }

  public void engage_lift() {
    // engages cylinders and disables operator control of the climbing motors
    // climbing motors MUST BE IN COAST while cylinder is extending
    // Use PID to set climber motors to assist cylinder???
    climberLiftSolenoid.set(false);
  }

  public void disengage_lift() {
    // default state of lift
    climberLiftSolenoid.set(true); //solenoid is inverted so true == retracted
  }

  public void stop() {
    /*
     * Turn off all motors.
     */

    climberMotorLeft.set(0.0);
    climberMotorRight.set(0.0);
  }

  public void go_to_position(double motorPos, boolean direction) {
    /*
     * Go to climber position using PID values.
     */

    climber_direction = direction;

    if (direction) {
      climberLiftSolenoid.set(true);
    }
    else {
      climberLiftSolenoid.set(false);
    }

    climberMotorLeftPID.setP(0.1);
    climberMotorLeftPID.setI(0.0);
    climberMotorLeftPID.setD(0.0);
    climberMotorLeftPID.setIZone(0.0);
    climberMotorLeftPID.setFF(0.0);
    climberMotorLeftPID.setOutputRange(-0.3, 0.3);
    climberMotorLeftPID.setReference(motorPos, ControlType.kPosition);

    climberMotorRightPID.setP(0.1);
    climberMotorRightPID.setI(0.0);
    climberMotorRightPID.setD(0.0);
    climberMotorRightPID.setIZone(0.0);
    climberMotorRightPID.setFF(0.0);
    climberMotorRightPID.setOutputRange(-0.3, 0.3);
    climberMotorRightPID.setReference(motorPos, ControlType.kPosition);
  }
}
