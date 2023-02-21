package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Elevator s_Elevator = new Elevator();
    private final Intake s_Intake = new Intake();
    private final Wrist s_Wrist = new Wrist();
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

    private final JoystickButton w_preset_0 = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton w_preset_1 = new JoystickButton(operator, XboxController.Button.kA.value);
    

    private final POVButton e_presButton_0 = new POVButton(operator, 0);
    private final POVButton e_presButton_1 = new POVButton(operator, 90);
    private final POVButton e_presButton_2 = new POVButton(operator, 180);

    private final JoystickButton intakeCone = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final double outtakeCone = operator.getRawAxis(XboxController.Axis.kLeftTrigger.value);
    private final JoystickButton intakeCube = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final double outtakeCube = operator.getRawAxis(XboxController.Axis.kRightTrigger.value);



    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);



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

        s_Wrist.setDefaultCommand(new TeleopWrist(s_Wrist, () -> -operator.getRawAxis(wristAxis)));

        s_Intake.setDefaultCommand(new TeleopIntake(s_Intake, () -> -outtakeCone, true));

        s_Intake.setDefaultCommand(new TeleopIntake(s_Intake, () -> -outtakeCube, false));
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


        w_preset_0.onTrue(new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT)));     
        w_preset_1.onTrue(new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_LOWER_LIMIT)));
        
        e_presButton_0.onTrue(new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorTop)));
        e_presButton_1.onTrue(new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorMid)));
        e_presButton_2.onTrue(new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorLow)));


        intakeCone.onTrue(new InstantCommand(() -> s_Intake.runIntake461(true)));
        intakeCone.onFalse(new InstantCommand(() -> s_Intake.stopIntake461()));
        intakeCone.whileFalse(Math.abs(outtakeCone) < Constants.stickDeadband ? new InstantCommand(() -> s_Intake.pulseIntake(false)) : new InstantCommand(() -> s_Intake.doNothing()));

        if (outtakeCone >= 0.2) {
            new InstantCommand(() -> s_Intake.runIntake461(false));
        } else if(outtakeCone < 0.2) {
            new InstantCommand(() -> s_Intake.stopIntake461());
        }

        intakeCube.onTrue(new InstantCommand(() -> s_Intake.runIntake461(false)));
        intakeCube.onFalse(new InstantCommand(() -> s_Intake.stopIntake461()));
        intakeCube.whileFalse(Math.abs(outtakeCube) < Constants.stickDeadband ? new InstantCommand(() -> s_Intake.pulseIntake(true)) : new InstantCommand(() -> s_Intake.doNothing()));

        if (outtakeCube >= 0.2) {
            new InstantCommand(() -> s_Intake.runIntake461(true));
        } else if (outtakeCube < 0.2) {
            new InstantCommand(() -> s_Intake.stopIntake461());
        }

        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }
    
    public void printValues(){
        SmartDashboard.putBoolean("Pov pressed", e_presButton_0.getAsBoolean());
        SmartDashboard.putNumber("Elevator Position", s_Elevator.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator Target", s_Elevator.getTarget());
        SmartDashboard.putBoolean("elevator limit triggered?", s_Elevator.elevatorSwitchTriggered());
        SmartDashboard.putNumber("Wrist Position", s_Wrist.getEncoder().getAbsolutePosition());
        SmartDashboard.putNumber("Wrist Target", s_Wrist.getTarget());
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
