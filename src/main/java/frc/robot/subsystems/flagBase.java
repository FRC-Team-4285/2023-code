package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.Constants.FlagConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class flagBase extends SubsystemBase{
    private CANSparkMax flagRaiser;

    private SparkMaxPIDController flagPID;
    private SparkMaxAbsoluteEncoder flagAbsEncoder;
    private RelativeEncoder flagNeoEncoder;

    public flagBase(RobotContainer robotContainer){
        flagRaiser = new CANSparkMax(FlagConstants.FLAG_MOTOR_ID, MotorType.kBrushless);
        flagRaiser.setIdleMode(IdleMode.kBrake);

        flagAbsEncoder = flagRaiser.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        flagAbsEncoder.setPositionConversionFactor(360.0); //assuming thru-bore encoder normally outputs in units of rotation

        //flagNeoEncoder = flagRaiser.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);

        flagPID = flagRaiser.getPIDController();
        flagPID.setFeedbackDevice(flagAbsEncoder);
        flagPID.setOutputRange(-180.0, 180.0);
        flagPID.setPositionPIDWrappingEnabled(true);
        flagPID.setP(1.0);
        flagPID.setI(0.01);
        flagPID.setD(0.0);
        //pid output = P * error + I * accumulated error + D * change in error
        //error = encoder output - target output
        //accumulated error = error + last error + error before that + error[-3] + error[-4] ...
        //change in error = error - last error
    }
    //now make it move
    public void moveFlag(double speed){
        flagRaiser.set(speed);
    }
    public void moveFlagToPosition(double angle){
        flagPID.setReference(angle, ControlType.kPosition); //this should actively drive the motor controller
        //if specific positions will be used in commands
        //add those positions as constants in Constants.java
        //make a subclass named after the subsystem those constants are specific to
    }
    //once the subsystem is complete, make commands that use the subsystem to perform the desired actions!
}
