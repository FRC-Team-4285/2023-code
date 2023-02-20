package frc.robot.subsystems;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.HardwareCAN;
import frc.robot.Constants.PneumaticChannels;

import com.revrobotics.CANSparkMax;
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
  private DutyCycleEncoder climberMotorEncoder;
  private Solenoid climberLiftSolenoid;

  public ClimberArmBase() {
    climberMotorLeft = new CANSparkMax(ClimberConstants.CLIMBER_MOTOR_LEFT_ID, MotorType.kBrushless);
    climberMotorRight = new CANSparkMax(ClimberConstants.CLIMBER_MOTOR_RIGHT_ID, MotorType.kBrushless);
    climberMotorEncoder = new DutyCycleEncoder(0);

    // This solenoid pumps the suction cup,
    // holding us to the balance station.
    climberLiftSolenoid = new Solenoid(
      HardwareCAN.PNEUMATIC_HUB,
      PneumaticsModuleType.REVPH,
      PneumaticChannels.CLIMBER_ARM_LIFTER
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

  public void engage_climber(boolean direction) {
    /*
     * Engage climber motors.
     */

    double power = ClimberConstants.CLIMBER_MOTOR_POWER;
    double pos = climberMotorEncoder.getAbsolutePosition();

    System.out.println("climb motor pos: " + pos);

    // ----------------------
    // !!! VERY IMPORTANT !!!
    // ----------------------
    // Each motor's power MUST be the opposite of each other.
    // If you don't, you will cause a large mechanical error
    // requiring an untimely and difficult manual fix.
    // 
    // More literally:
    //   - If left is power, right must be -power;
    //   - If left is -power, right must be power.
    //
    // You have been warned. If you break it, YOU fix it!
    // Yes, that means YOU.

    if (direction) {
        climberLiftSolenoid.set(true);
        climberMotorLeft.set(power);
        climberMotorRight.set(-power);
    }
    else {
        climberLiftSolenoid.set(false);
        climberMotorLeft.set(-power);
        climberMotorRight.set(power);
    }
  }

  public void engage_lift(){
    // engages cylinders and disables operator control of the climbing motors
    // climbing motors MUST BE IN COAST while cylinder is extending
    // Use PID to set climber motors to assist cylinder???
    climberLiftSolenoid.set(false);

  }

  public void disengage_lift(){
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
}
