package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase{
    private CANSparkMax wrist;
    private PIDController wristPidController = new PIDController(Constants.WRIST_P, Constants.WRIST_I, Constants.WRIST_D);
    private DutyCycleEncoder wristEncoder;
    double position = 0;
    double target = 0;

    public Wrist() {
        wrist = new CANSparkMax(32, MotorType.kBrushless);
        wristEncoder = new DutyCycleEncoder(2);
        wrist.restoreFactoryDefaults();
        wrist.setInverted(false);
        wrist.setSmartCurrentLimit(30);
        wrist.set(wristPidController.calculate(wristEncoder.getAbsolutePosition(), Constants.WRIST_UPPER_LIMIT));
    }

    public DutyCycleEncoder getEncoder() {
        return wristEncoder;
    }

    public double getTarget() {
        return target;
    }

    public void setRotation(double rotation) {
        if (rotation < wristEncoder.getAbsolutePosition() && wristEncoder.getAbsolutePosition() < Constants.WRIST_LOWER_LIMIT) {
            rotation = Constants.WRIST_LOWER_LIMIT;
        } else if (rotation > wristEncoder.getAbsolutePosition() && wristEncoder.getAbsolutePosition() > Constants.WRIST_UPPER_LIMIT) {
            rotation = Constants.WRIST_UPPER_LIMIT;
        }
        target = rotation;
        wrist.set(wristPidController.calculate(wristEncoder.getAbsolutePosition(), rotation));
    }

}
