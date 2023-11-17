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
    DigitalInput elevatorSwitch = new DigitalInput(3); //limit switch that re-zeros the elevator encoder;

    public Elevator() {
        elevator = new CANSparkMax(31, MotorType.kBrushless);
        m_encoder = elevator.getEncoder();
        elevator.restoreFactoryDefaults();
        elevator.setSmartCurrentLimit(70);
        elevator.setInverted(true);

    }
    
    public RelativeEncoder getEncoder() {
        return m_encoder;
    }

    public double getTarget() {
        return target;
    }
    public double elevatorPower() {
        return elevator.getAppliedOutput();
    }
    
    //goto a preset

    public void setElevatorPreset(double target){
        elevator.set(pidController.calculate(m_encoder.getPosition(), target));
    }

    public void setHeight(double height) {
        System.out.print("setting height");
        if (height < m_encoder.getPosition() && elevatorSwitchTriggered()) {
            m_encoder.setPosition(0);
            height = 0;
        } else if (height > m_encoder.getPosition() && m_encoder.getPosition() > Constants.ELEVATOR_UPPER_LIMIT) {
            height = Constants.ELEVATOR_UPPER_LIMIT;
        }
        elevator.set(pidController.calculate(m_encoder.getPosition(), height));
        target = height;
    }

    public void moveElevator(double movementVector)
    {
        
        if(movementVector < 0 && elevatorSwitchTriggered())
        {
            target = 0;
            holdHeight();
           return;
        }
        else if (movementVector > 0 && m_encoder.getPosition() > Constants.ELEVATOR_UPPER_LIMIT)
        {
            target = Constants.ELEVATOR_UPPER_LIMIT;
            holdHeight();
            return;
        }
        if(elevatorSwitchTriggered())
        {
            m_encoder.setPosition(0); //was zero (don't worry about it);
        }
        elevator.set(movementVector);
        target = m_encoder.getPosition();
    }

    public void holdHeight()
    {
        elevator.set(pidController.calculate(m_encoder.getPosition(), target));
    }

    public boolean elevatorSwitchTriggered() {
        return !elevatorSwitch.get();
    }

    public void checkLimitSwitches()
    {
        if(elevatorSwitchTriggered())
        {
            m_encoder.setPosition(0);
        }

    }
}
