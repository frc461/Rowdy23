package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase{
    private CANSparkMax wrist;
    private PIDController wristPidController = new PIDController(Constants.WRIST_P, Constants.WRIST_I, Constants.WRIST_D);
    private SparkMaxAbsoluteEncoder wristEncoder;
    private SparkMaxLimitSwitch forwardLimitSwitch;
    private SparkMaxLimitSwitch reverseLimitSwitch;

    double target = Constants.WRIST_UPPER_LIMIT;

    public Wrist(RelativeEncoder _encoder) {
        wrist = new CANSparkMax(32, MotorType.kBrushless);
        //wristEncoder = new DutyCycleEncoder(2);
        wrist.restoreFactoryDefaults();
        wrist.setInverted(false);
        wrist.setSmartCurrentLimit(30);
        wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);
        //wrist.set(wristPidController.calculate(wristEncoder.getAbsolutePosition(), Constants.WRIST_UPPER_LIMIT));
        forwardLimitSwitch = wrist.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        reverseLimitSwitch = wrist.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    }

    public SparkMaxAbsoluteEncoder getEncoder() {
        return wristEncoder;
    }

    public double getTarget() {
        return target;
    }

    public void setRotation(double rotation) {
        if (rotation < wristEncoder.getPosition() && wristEncoder.getPosition() < Constants.WRIST_LOWER_LIMIT) {
            rotation = Constants.WRIST_LOWER_LIMIT;
        } else if (rotation > wristEncoder.getPosition() && wristEncoder.getPosition() > Constants.WRIST_UPPER_LIMIT) {
            rotation = Constants.WRIST_UPPER_LIMIT;
        }
        target = rotation;
        wrist.set(wristPidController.calculate(wristEncoder.getPosition(), rotation));
    }

    public void moveWrist(double movementVector)
    {
        double wristPracticalLowerLimit = Constants.WRIST_LOWER_LIMIT; //+ calculateBottomLimit();
        if(movementVector < 0 && wristEncoder.getPosition() < wristPracticalLowerLimit)
        {
            target = wristPracticalLowerLimit;
            holdWrist();
            return;
        }
        else if (movementVector > 0 && wristEncoder.getPosition() > Constants.WRIST_UPPER_LIMIT)
        {
            target = Constants.WRIST_UPPER_LIMIT;
            holdWrist();
            return;
        }

        wrist.set(movementVector);
        target = wristEncoder.getPosition();

    }

    public void holdWrist()
    {
        double wristPracticalLowerLimit = Constants.WRIST_LOWER_LIMIT; //+ calculateBottomLimit();
        if(target < wristPracticalLowerLimit){
            target = wristPracticalLowerLimit;
        }
        wrist.set(wristPidController.calculate(wristEncoder.getPosition(), target));
    }

    public boolean getForwardLimitSwitch()
    {
        return forwardLimitSwitch.isPressed();
    }

    public boolean getReverseLimitSwitch()
    {
        return reverseLimitSwitch.isPressed();
    }

    // private double calculateBottomLimit(){
    //     double adjustment = 0;
    //     if(elevatorEncoder.getPosition() < 10){
    //         adjustment = 0.1 * (1 - (elevatorEncoder.getPosition()/10));
    //     }

    //     return adjustment;
    // }

}
