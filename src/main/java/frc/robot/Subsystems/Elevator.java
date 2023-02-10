package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator {
    private CANSparkMax elevator = new CANSparkMax(31, MotorType.kBrushed);
    

    public void up(){
        elevator.set(1);
    }
    
    public void down(){
        elevator.set(-1);
    }
    
    public void elevatorInit(){
        elevator.getPIDController().setP(Constants.ELEVATOR_P);
        elevator.getPIDController().setI(Constants.ELEVATOR_I);
        elevator.getPIDController().setD(Constants.ELEVATOR_D);
        elevator.getPIDController().setFF(Constants.ELEVATOR_FF);
    
    }
}




