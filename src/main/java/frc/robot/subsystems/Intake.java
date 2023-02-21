package frc.robot.subsystems;
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
    }

    private AddressableLED led = new AddressableLED(0);
    private AddressableLEDBuffer ledData = new AddressableLEDBuffer(13);

    
    //run intake except its actually good
    public void runIntake461(boolean reversed) {
        if (!reversed) {
            intake.set(0.7);
            showLights(255, 0, 255);
        } else {
            intake.set(-0.7);
            showLights(255, 200, 0);
        }
    }

    public void pulseIntake (boolean reversed) {
        if (coneBeamBroken() || cubeBeamBroken()) {
            if (!reversed) {
                intake.set(0.1);
                Timer.delay(1);
            } else {
                intake.set(-0.1);
                Timer.delay(1);
            }
        } else if (!coneBeamBroken() && !cubeBeamBroken()) {
            intake.set(0);
        }
    }

    public void doNothing() {}

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


    public boolean cubeBeamBroken(){
        return cubeBeam.get();
    }

    public boolean coneBeamBroken() {
        return coneBeam.get();
    }
}
