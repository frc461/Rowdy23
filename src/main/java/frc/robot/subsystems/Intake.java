package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax intake;
    DigitalInput cubeBeam = new DigitalInput(0);
    DigitalInput coneBeam = new DigitalInput(1);
    private int counter = 0;
    // public Lights lights = new Lights();
    
    private AddressableLED led = new AddressableLED(0);
    private AddressableLEDBuffer ledData = new AddressableLEDBuffer(2);

    private DigitalOutput intakeIndicator = new DigitalOutput(4);

    public boolean intakeCube = false;

    public Intake() {
        intake = new CANSparkMax(33, MotorType.kBrushed);
        intake.restoreFactoryDefaults();
        intake.setInverted(true);
        led.setLength(ledData.getLength());
        //lights.showLights("blue");
        showLights(255, 0, 0);
        
    }
    //3553 pepvlldrv

    public void turnOnIndicator() { intakeIndicator.set(true); }

    public void turnOffIndicator() { intakeIndicator.set(false); }

    public boolean cubeBeamBroken() { return cubeBeam.get(); }

    public boolean coneBeamBroken() { return coneBeam.get(); }

    public void setSpeed(double speed) { intake.set(speed); }
    
    public double getSpeed() { return intake.get(); }

    public void runIntake(Joystick joystick){
        if (joystick.getRawButton(XboxController.Button.kRightBumper.value)) {
            //lights.showLights("yellow");
            showLights(255, 0, 255);
            intake.set(-1);
            intakeCube = false;
        } else if (joystick.getRawButton(XboxController.Button.kLeftBumper.value)) {
            intake.set(0.7);
            //lights.showLights("purple");
            showLights(255, 255, 0);
            intakeCube = true;
        } else if (joystick.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.2) {
            intake.set(0.65);
        } else if (joystick.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.2) {
            intake.set(-0.7);
        } else if ((intakeCube && cubeBeamBroken()) && (!joystick.getRawButton(XboxController.Button.kLeftBumper.value) && !joystick.getRawButton(XboxController.Button.kRightBumper.value))) {
            pulseIntake(0.1);
            System.out.println("cubeBeam"+ intakeCube);
        } else if ((!intakeCube && coneBeamBroken()) && (!joystick.getRawButton(XboxController.Button.kLeftBumper.value) && !joystick.getRawButton(XboxController.Button.kRightBumper.value))) {
            pulseIntake(-0.1);
            System.out.println("coneBeam:" + intakeCube);
        } else {
            intake.set(0);
            //lights.showLights("red");
            // stopLights();
        }

        // beam brake indicator. beam brake lmao
        if (cubeBeamBroken() || coneBeamBroken()) { turnOnIndicator(); } else { turnOffIndicator(); }
    }

    public void pulseIntake(double speed){
        if(counter > 10000) { counter = 0; }
        if(counter++ < 5000) { intake.set(speed); }
    }

    public void showLights(int r, int g, int b) {
        for (int i = 0; i < ledData.getLength(); i++) {
            ledData.setRGB(i, r, g, b);
        }
        led.setData(ledData);
        led.start();

    }

    public void stopLights() {
        for (int i = 0; i < ledData.getLength(); i++) {
            ledData.setRGB(i, 0, 0, 0);
        }
        led.setData(ledData);
        led.start();
    }
}
