package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {
    private CANSparkMax elevator = new CANSparkMax(31, MotorType.kBrushed);
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    public double setPoint, processVariable;

    public void up(){
        setPoint = 1;
        m_pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
        processVariable = m_encoder.getPosition();
    }

    public void mid() {
        setPoint = 0.5;
        m_pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
        processVariable = m_encoder.getPosition();
    }

    public void down(){
        setPoint = 0;
        m_pidController.setReference(-1, CANSparkMax.ControlType.kVelocity);
    }
    
    public void elevatorInit(){
        m_pidController = elevator.getPIDController();
        m_encoder = elevator.getEncoder();

        // PID coefficients
        // kP = 5e-5; 
        // kI = 1e-6;
        // kD = 0; 
        kIz = 0; 
        kFF = 0.000156; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;

        // Smart Motion Coefficients
        maxVel = 2000; // rpm
        maxAcc = 1500;      
        
        m_pidController.setP(Constants.ELEVATOR_P);
        m_pidController.setI(Constants.ELEVATOR_I);
        m_pidController.setD(Constants.ELEVATOR_D);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        int smartMotionSlot = 0;
        m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    }

}




