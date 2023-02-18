package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    private CANSparkMax intake = new CANSparkMax(33, MotorType.kBrushed);
    private CANSparkMax wrist = new CANSparkMax(32, MotorType.kBrushed);
    private PIDController wristPidController = new PIDController(Constants.WRIST_P, Constants.WRIST_I, Constants.WRIST_D); 
    //private AbsoluteEncoder wristEncoder = intake.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(2);
    double position = 0;
    DigitalInput cubeBeam = new DigitalInput(0);
    DigitalInput coneBeam = new DigitalInput(1);

    private AddressableLED led = new AddressableLED(0);
    private AddressableLEDBuffer ledData = new AddressableLEDBuffer(13);


    

    
    public void stop() {
        
        wrist.set(wristPidController.calculate(wristEncoder.getAbsolutePosition(), position));
    }

    public void setRotation(double a) {
        position+=a/50;
        // if(wristEncoder.getAbsolutePosition() > 0 && wristEncoder.getAbsolutePosition() < 0.9375){
        // }
        // else{
        //     stop();
        // }
        double targetPos = 0.8;
        
        if(position < .95 && position > 0.5){
            targetPos = position;
        }
        if(wristEncoder.getAbsolutePosition() < .95){
            targetPos = 0.945;
        }
        else if(wristEncoder.getAbsolutePosition() > 0.5){
            targetPos = 0.5;
        }

        
        SmartDashboard.putNumber("wrist power", wristPidController.calculate(wristEncoder.getAbsolutePosition(), targetPos));
        
        wrist.set(wristPidController.calculate(wristEncoder.getAbsolutePosition(), targetPos));
            SmartDashboard.putNumber("wrist target", targetPos);
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

    public void printValues() {
        SmartDashboard.putNumber("wrist enc", wristEncoder.getAbsolutePosition());
    }
    public boolean cubeBeamBroken(){
        return cubeBeam.get();
    }
}
