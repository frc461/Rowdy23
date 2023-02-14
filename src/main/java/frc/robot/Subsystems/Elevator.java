package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {
    private CANSparkMax elevator = new CANSparkMax(31, MotorType.kBrushless);
    //private SparkMaxPIDController m_pidController = elevator.getPIDController();
    private PIDController pidController = new PIDController(Constants.ELEVATOR_P, Constants.ELEVATOR_I, Constants.ELEVATOR_D);
    private RelativeEncoder m_encoder = elevator.getEncoder();
    private static final int elevatorMax = 1024;
    double position = 0; 
    

    
    public void up(){
        elevator.set(pidController.calculate(m_encoder.getPosition(), 1000));
    }

    public void mid() {
        elevator.set(pidController.calculate(m_encoder.getPosition(), 0));
    }

    public void down(){
        elevator.set(pidController.calculate(m_encoder.getPosition(), -400));
    }

    public void stop(){
        elevator.set(pidController.calculate(m_encoder.getPosition(), position));
    }
        
    public void moveSlow(double stickY) {
        if(m_encoder.getPosition() < elevatorMax){
            position += stickY*30;
            elevator.set(pidController.calculate(m_encoder.getPosition(), position));
        }
        else{
            stop();
        }
    }

}




