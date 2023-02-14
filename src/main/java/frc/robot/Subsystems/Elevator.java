package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
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
    

    
    // public void up() {
    //     //elevator.getPIDController().setReference(10, CANSparkMax.ControlType.kPosition);
    //     if(m_encoder.getPosition() < 1000*4) {
    //         elevator.getPIDController().setReference(-0.8, CANSparkMax.ControlType.kVelocity);
    //     } else {
    //         elevator.set(0);
    //     }

    // }

    // public void mid() {
    //     if(m_encoder.getPosition() < 512*4) {
    //         elevator.getPIDController().setReference(-30, CANSparkMax.ControlType.kVelocity);
    //         //elevator.getPIDController().setReference(5, CANSparkMax.ControlType.kPosition);
    //     } else if(m_encoder.getPosition() > 512*4){
    //         elevator.getPIDController().setReference(30, CANSparkMax.ControlType.kVelocity);
    //        // elevator. getPIDController().setReference(-5, CANSparkMax.ControlType.kPosition);
    //     } else {
    //         elevator.set(0);
    //     }
    // }

    // public void down(){
    //     if(m_encoder.getPosition() > 0) {
    //         elevator.getPIDController().setReference(30, CANSparkMax.ControlType.kVelocity);
    //     } else {
    //         elevator.set(0);
    //     }
    //     //elevator.getPIDController().setReference(1, CANSparkMax.ControlType.kPosition);
    // }
    public void stop(){
        elevator.set(pidController.calculate(m_encoder.getPosition(), position));
    }
        
    public void moveSlow(double stickY) {
        position += stickY;
        elevator.set(pidController.calculate(m_encoder.getPosition(), position));
    }

    public void elevatorInit(){
        // m_pidController = elevator.getPIDController();
    
        // m_pidController.setP(Constants.ELEVATOR_P);
        // m_pidController.setI(Constants.ELEVATOR_I);
        // m_pidController.setD(Constants.ELEVATOR_D);

        
        

    }

}




