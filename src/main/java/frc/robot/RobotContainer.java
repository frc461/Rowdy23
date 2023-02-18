package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    //op controls
    private final int wristAxis = XboxController.Axis.kRightY.value;
    private final int elevatorAxis = XboxController.Axis.kLeftY.value;

    private final JoystickButton e_preset_0 = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton e_preset_1 = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton e_preset_2 = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton e_preset_3 = new JoystickButton(operator, XboxController.Button.kX.value);

    private final JoystickButton intakeCone = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final boolean outtakeCone = operator.getRawAxis(XboxController.Axis.kLeftTrigger.value) >= 0.2 ? true : false;
    private final JoystickButton intakeCube = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final boolean outtakeCube = operator.getRawAxis(XboxController.Axis.kRightTrigger.value) >= 0.2 ? true : false;


    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Elevator s_Elevator = new Elevator();
    private final Intake s_Intake = new Intake();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        s_Elevator.setDefaultCommand(new TeleopElevator(s_Elevator, () -> -operator.getRawAxis(elevatorAxis)));

        s_Intake.setDefaultCommand(new TeleopWrist(s_Intake, () -> -operator.getRawAxis(wristAxis)));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons (and op buttons)*/


        e_preset_0.onTrue(new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorTop)));     
        e_preset_1.onTrue(new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorMid)));
        e_preset_2.onTrue(new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorLow)));     
        e_preset_3.onTrue(new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)));

        intakeCone.onTrue(new InstantCommand(() -> s_Intake.runIntake461(true)));
        intakeCone.onFalse(new InstantCommand(() -> s_Intake.stopIntake461()));
        if (outtakeCone) {
            new InstantCommand(() -> s_Intake.runIntake461(false));
        } else if (!outtakeCone) {
            new InstantCommand(() -> s_Intake.stopIntake461());
        }
        intakeCube.onTrue(new InstantCommand(() -> s_Intake.runIntake461(false)));
        intakeCube.onFalse(new InstantCommand(() -> s_Intake.stopIntake461()));

        if (outtakeCube) {
            new InstantCommand(() -> s_Intake.runIntake461(true));
        } else if (!outtakeCube) {
            new InstantCommand(() -> s_Intake.stopIntake461());
        }


        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }
    
    public void printValues(){
        SmartDashboard.putNumber("Elevator Position", s_Elevator.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator Target", s_Elevator.getTarget());
        SmartDashboard.putBoolean("elevator limit triggered?", s_Elevator.elevatorSwitchTriggered());
        SmartDashboard.putNumber("Wrist Position", s_Intake.getEncoder().getAbsolutePosition());
        SmartDashboard.putNumber("Wrist Target", s_Intake.getTarget());
        SmartDashboard.putBoolean("cube beam broken?: ", s_Intake.cubeBeamBroken());
        SmartDashboard.putBoolean("cone beam broken?", s_Intake.coneBeamBroken());
    }

    // private void stow(){
    //     s_Elevator.setElevatorPreset(Constants.elevatorStowPos);
    //     s_Intake.setRotation(Constants.wristStowPos);
    // }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
