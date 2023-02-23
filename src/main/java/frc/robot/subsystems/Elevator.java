package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private CANSparkMax elevator;
    private PIDController pidController = new PIDController(Constants.ELEVATOR_P, Constants.ELEVATOR_I, Constants.ELEVATOR_D);
    private RelativeEncoder m_encoder;
    double position = 0;
    double target = 0;
    DigitalInput elevatorSwitch = new DigitalInput(3);

    public Elevator() {
        elevator = new CANSparkMax(31, MotorType.kBrushless);
        m_encoder = elevator.getEncoder();
        elevator.restoreFactoryDefaults();
        elevator.setSmartCurrentLimit(30);
        elevator.setInverted(true);
    }
    
    public RelativeEncoder getEncoder() {
        return m_encoder;
    }

    public double getTarget() {
        return target;
    }
    
    //goto a preset

    public void setElevatorPreset(double target){
        elevator.set(pidController.calculate(m_encoder.getPosition(), target));
    }

    public void setHeight(double height) {
        if (height < m_encoder.getPosition() && elevatorSwitchTriggered()) {
            m_encoder.setPosition(0);
            height = 0;
        } else if (height > m_encoder.getPosition() && m_encoder.getPosition() > Constants.ELEVATOR_UPPER_LIMIT) {
            height = Constants.ELEVATOR_UPPER_LIMIT;
        }
        elevator.set(pidController.calculate(m_encoder.getPosition(), height));
        target = height;
    }

    public boolean elevatorSwitchTriggered() {
        return !elevatorSwitch.get();
    }
}
