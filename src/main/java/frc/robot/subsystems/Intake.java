package frc.robot.subsystems;
import java.sql.Time;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax intake;
    DigitalInput cubeBeam = new DigitalInput(0);
    DigitalInput coneBeam = new DigitalInput(1);

    public Intake() {
        intake = new CANSparkMax(33, MotorType.kBrushed);
        showLights(255, 0, 0);
    }


    private AddressableLED led = new AddressableLED(0);
    private AddressableLEDBuffer ledData = new AddressableLEDBuffer(13);


    public boolean cubeBeamBroken(){
        return cubeBeam.get();
    }

    public boolean coneBeamBroken() {
        return coneBeam.get();
    }

    public void runIntake(double speed, boolean joystick){
        System.out.println(speed);
       intake.set(speed);
    }

    public void pulseIntake(double speed){
        intake.set(speed);
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
