package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxPIDController;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    private CANSparkMax intake = new CANSparkMax(33, MotorType.kBrushed);
    private CANSparkMax wrist = new CANSparkMax(32, MotorType.kBrushed);
    private PIDController wristPidController = new PIDController(Constants.WRIST_P, Constants.WRIST_I, Constants.WRIST_D); 
    private AbsoluteEncoder wristEncoder = intake.getEncoder();
    double position = 0;

    private AddressableLED led = new AddressableLED(0);
    private AddressableLEDBuffer ledData = new AddressableLEDBuffer(13);
    
    public void stop() {
        wrist.set(wristPidController.calculate(wristEncoder.getPosition()));
    }

    public void turn(double a) {
        position += a;
        wrist.set(wristPidController.calculate(wristEncoder.getPosition(), position));
    }

    public Intake(){
        
        // intakeMotor.setSmartCurrentLimit(55);
        // shooterFeedMotor.setSmartCurrentLimit(45);
        // shooterFeedMotor.setOpenLoopRampRate(0.5);
        // intakeMotor.burnFlash();
        // shooterFeedMotor.burnFlash(); 
        
             
    }

    //run intake except its actually good
    public void runIntake461(boolean reversed) {
        if (!reversed) {
            intake.set(0.5);
           

        } else {
            intake.set(-0.5);
        }
    }

    public void stopIntake461() {
        
        intake.set(0);
       
    }

    public void showLights(int r, int g, int b) {
        led.setLength(ledData.getLength());
        for (int i = 0; i < ledData.getLength(); i++) {
            ledData.setRGB(i, r, g, b);
        }
        led.setData(ledData);
        led.start();
    }

    public void stopLights() {
        led.setLength(ledData.getLength());
        for (int i = 0; i < ledData.getLength(); i++) {
            ledData.setRGB(i, 0, 0, 0);
        }
        led.setData(ledData);
        led.start();
    }
}
