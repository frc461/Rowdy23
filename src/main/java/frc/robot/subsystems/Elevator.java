package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator {
    private CANSparkMax elevator = new CANSparkMax(31, MotorType.kBrushless);
    //private SparkMaxPIDController m_pidController = elevator.getPIDController();
    private PIDController pidController = new PIDController(Constants.ELEVATOR_P, Constants.ELEVATOR_I, Constants.ELEVATOR_D);
    private RelativeEncoder m_encoder = elevator.getEncoder();
    private static final int elevatorMax = 1024;
    double position = 0;
    double target = 0;
    double power = 0;
    DigitalInput elevatorSwitch = new DigitalInput(3);
    
    public RelativeEncoder getEncoder() {
        return m_encoder;
    }

    public double getTarget() {
        return target;
    }

    public double getPower() {
        return power;
    }
    
    //goto a preset

    public void setElevatorPreset(double target){
        elevator.set(pidController.calculate(m_encoder.getPosition(), target));
    }

    public void stop(){
        elevator.set(pidController.calculate(m_encoder.getPosition(), position));
    }

    public void moveSlow(double stickY) {
        if(m_encoder.getPosition() < elevatorMax){
            if (elevatorSwitch.get()) {
                m_encoder.setPosition(0);
                position = 0;
                target = 0;
            } else {
                position += stickY*30;
                power = pidController.calculate(m_encoder.getPosition(), position);
                elevator.set(power);
                target = position;
            }
        }
        else{
            stop();
        }
    }

    public boolean elevatorSwitchTriggered() {
        return elevatorSwitch.get();
    }
}




