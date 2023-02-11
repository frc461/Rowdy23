package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {
    private CANSparkMax elevator = new CANSparkMax(31, MotorType.kBrushless);
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;
    private static final int elevatorMax = 1024;
    
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
        elevator.set(0);
    }
        
    public void slowUp() {
        //if(m_encoder.getPosition() < elevatorMax*4) {
            //elevator.getPIDController().setReference(-15, CANSparkMax.ControlType.kVelocity);
        //} else {
            elevator.set(-0.8);
        //}
    }

    public void slowDown() {
        // if(m_encoder.getPosition() > 0) {
        //     elevator.getPIDController().setReference(15, CANSparkMax.ControlType.kVelocity);
        // } else {
             elevator.set(0.2);
        // }
    }
    public void elevatorInit(){
        // m_pidController = elevator.getPIDController();
        // m_encoder = elevator.getEncoder();
        
        // m_pidController.setP(Constants.ELEVATOR_P);
        // m_pidController.setI(Constants.ELEVATOR_I);
        // m_pidController.setD(Constants.ELEVATOR_D);
        

    }

}




