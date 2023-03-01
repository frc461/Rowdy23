package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax intake;
    DigitalInput cubeBeam = new DigitalInput(0);
    DigitalInput coneBeam = new DigitalInput(1);
    private int counter = 0;
    private AddressableLED led = new AddressableLED(4);
    //private AddressableLED led2 = new AddressableLED(5);

    private AddressableLEDBuffer ledData = new AddressableLEDBuffer(13);

    //private AddressableLEDBuffer ledData2 = new AddressableLEDBuffer(13);

    public Intake() {
        intake = new CANSparkMax(33, MotorType.kBrushed);
        intake.restoreFactoryDefaults();
        intake.setInverted(true);
        led.setLength(ledData.getLength());
        showLights(255, 0, 0);
    }


    public boolean cubeBeamBroken(){
        return cubeBeam.get();
    }

    public boolean coneBeamBroken() {
        return coneBeam.get();
    }

    public void runIntake(Joystick joystick){
        if (joystick.getRawButton(XboxController.Button.kRightBumper.value)) {
            showLights(255, 0, 255);
            intake.set(0.7);
        } else if (joystick.getRawButton(XboxController.Button.kLeftBumper.value)) {
            intake.set(-0.7);
            showLights(255, 255, 0);
        } else if (joystick.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.2) {
            intake.set(-0.2);
        } else if (joystick.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.2) {
            intake.set(0.7);
        } else if (coneBeamBroken() == true && !joystick.getRawButton(XboxController.Button.kLeftBumper.value) && !joystick.getRawButton(XboxController.Button.kRightBumper.value)){
            pulseIntake(.1);
        } else if (cubeBeamBroken() == true && !joystick.getRawButton(XboxController.Button.kLeftBumper.value) && !joystick.getRawButton(XboxController.Button.kRightBumper.value)){
            pulseIntake(-.1);
        } else {
            intake.set(0);
            showLights(255, 0, 0);
        }
    }

    public void pulseIntake(double speed){
        if(counter > 10000) { counter = 0; }
        if(counter++ < 5000) { intake.set(speed); }
    }

    public void setSpeed(double speed) {
        intake.set(speed);
        
    }

    public double getSpeed() {
        return intake.get();
    }


    public void showLights(int r, int g, int b) {
        
        //led2.setLength(ledData.getLength());
        for (int i = 0; i < ledData.getLength(); i++) {
            ledData.setRGB(i, r, g, b);
            //ledData2.setRGB(i, r, g, b);
        }
        led.setData(ledData);
        //led2.setData(ledData);

        led.start();
        //led2.start();

    }

    public void stopLights() {

        //led2.setLength(ledData.getLength());
        for (int i = 0; i < ledData.getLength(); i++) {
            ledData.setRGB(i, 0, 0, 0);
            //ledData2.setRGB(i, 0, 0, 0);
        }
        led.setData(ledData);
        //led2.setData(ledData);

        led.start();
        //led2.start();

    }

   
}
