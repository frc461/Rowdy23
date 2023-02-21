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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax intake;
    private CANSparkMax wrist;
    private PIDController wristPidController = new PIDController(Constants.WRIST_P, Constants.WRIST_I, Constants.WRIST_D);
    private DutyCycleEncoder wristEncoder;
    double position = 0;
    double target = 0;
    DigitalInput cubeBeam = new DigitalInput(0);
    DigitalInput coneBeam = new DigitalInput(1);

    public Intake() {
        intake = new CANSparkMax(33, MotorType.kBrushed);
        wrist = new CANSparkMax(32, MotorType.kBrushed);
        wristEncoder = new DutyCycleEncoder(2);
        wrist.restoreFactoryDefaults();
        wrist.setInverted(false);
        wrist.set(wristPidController.calculate(wristEncoder.getAbsolutePosition(), 0.38));
    }

    private AddressableLED led = new AddressableLED(0);
    private AddressableLEDBuffer ledData = new AddressableLEDBuffer(13);

    public DutyCycleEncoder getEncoder() {
        return wristEncoder;
    }

    public double getTarget() {
        return target;
    }

    public void setRotation(double rotation) {
        if (rotation < wristEncoder.getAbsolutePosition() && wristEncoder.getAbsolutePosition() < Constants.WRIST_LOWER_LIMIT) {
            rotation = Constants.WRIST_LOWER_LIMIT;
        } else if (rotation > wristEncoder.getAbsolutePosition() && wristEncoder.getAbsolutePosition() > Constants.WRIST_UPPER_LIMIT) {
            rotation = Constants.WRIST_UPPER_LIMIT;
        }
        target = rotation;
        wrist.set(wristPidController.calculate(wristEncoder.getAbsolutePosition(), target));
    }

    //run intake except its actually good
    public void runIntake461(boolean reversed) {

        if (!reversed) {
            intake.set(0.5);
            showLights(255, 0, 255);
        } else {
            intake.set(-0.5);
            showLights(255, 200, 0);
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

    public boolean coneBeamBroken() {
        return coneBeam.get();
    }
}
