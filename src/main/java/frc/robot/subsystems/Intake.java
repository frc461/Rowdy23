package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax intake;
    DigitalInput cubeBeam = new DigitalInput(0);
    DigitalInput coneBeam = new DigitalInput(1);

    public Intake() {
        intake = new CANSparkMax(33, MotorType.kBrushed);
        intake.restoreFactoryDefaults();
        intake.setInverted(true);
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

    public void runIntake(Joystick joystick){
        //System.out.println(speed);
        if(joystick.getRawButton(XboxController.Button.kRightBumper.value)) {
            intake.set(0.7);
        } else if (joystick.getRawButton(XboxController.Button.kLeftBumper.value)) {
            intake.set(-0.7);
        } else if (joystick.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.2) {
            intake.set(-0.2);
        } else if (joystick.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.2) {
            intake.set(0.2);
        } else if(coneBeamBroken() == true && !joystick.getRawButton(XboxController.Button.kLeftBumper.value) && !joystick.getRawButton(XboxController.Button.kRightBumper.value)){
            pulseIntake(.1);
            System.out.println("pulse1");
        } else if(cubeBeamBroken() == true && !joystick.getRawButton(XboxController.Button.kLeftBumper.value) && !joystick.getRawButton(XboxController.Button.kRightBumper.value)){
            pulseIntake(-.1);
            System.out.println("pulse2");
        } else {
            intake.set(0);
        }
    }

    public void pulseIntake(double speed){
        intake.set(speed);
        Timer.delay(1);
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
