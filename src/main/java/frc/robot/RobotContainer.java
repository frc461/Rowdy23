package frc.robot;

import java.util.HashMap;
import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.*;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.commands.autocommands.AutoIntake;
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

    private String pPlan = null;

    public CommandBase autoCode = Commands.sequence(new PrintCommand("no auto selected"));

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    //op controls
    private final int wristAxis = XboxController.Axis.kLeftY.value;
    private final int elevatorAxis = XboxController.Axis.kRightY.value;



    private final JoystickButton e_presButton_0 = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton e_presButton_1 = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton e_presButton_2 = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton e_presButton_3 = new JoystickButton(operator, XboxController.Button.kB.value);


    private final POVButton w_preset_0 = new POVButton(operator, 0);
    private final POVButton w_preset_1 = new POVButton(operator, 90);
    private final POVButton w_preset_2 = new POVButton(operator, 180);
    private final POVButton operator_stowButton = new POVButton(operator, 270);


    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driver_stowButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton driver_AutoBalance = new JoystickButton(driver, XboxController.Button.kB.value);

    private final POVButton driver_stowButton2 = new POVButton(operator, 270);
    // private final JoystickButton xModeButton = new JoystickButton(driver, XboxController.Button.kX.value);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> driver.getRawAxis(rotationAxis),
                robotCentric::getAsBoolean
            )
        );

        s_Elevator.setDefaultCommand(
            new TeleopElevator(
                s_Elevator,
                () -> -operator.getRawAxis(elevatorAxis)
            )
        );

        s_Wrist.setDefaultCommand(
            new TeleopWrist(
                s_Wrist,
                () -> -operator.getRawAxis(wristAxis)
            )
        );

        s_Intake.setDefaultCommand(
            new TeleopIntake(
                s_Intake,
                operator
            )
        );

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
        /* Driver Buttons (and op buttons) */

        w_preset_0.onTrue(new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT)));
        w_preset_1.onTrue(new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_MID_LIMIT)));
        w_preset_2.onTrue(new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_LOWER_LIMIT)));

        e_presButton_0.onTrue( // Preset to score high cone
            Commands.sequence(
                new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorHighScore)),
                new WaitCommand(.75),
                new InstantCommand(() -> s_Wrist.setRotation(Constants.wristHighConeScore))
            )
        );

        e_presButton_1.onTrue( // Preset to score mid cone
            Commands.sequence(
                new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorMidScore)),
                new WaitCommand(.25),
                new InstantCommand(() -> s_Wrist.setRotation(Constants.wristMidConeScore))
            )
        );

        e_presButton_2.onTrue( // Preset to pick up fallen cone
            Commands.sequence(
                new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorConePickup)),
                new InstantCommand(() -> s_Wrist.setRotation(Constants.wristConePickup))
            )
        );

        e_presButton_3.onTrue( // Preset to pick up cone from single substation
            Commands.sequence(
                new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)),
                new InstantCommand(() -> s_Wrist.setRotation(Constants.wristConePickup2))
            )
        );

        w_preset_0.onTrue( // Preset to score high cube
            Commands.sequence(
                new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorHighScore)),
                new WaitCommand(.75),
                new InstantCommand(() -> s_Wrist.setRotation(Constants.wristHighCubeScore))
            )
        );

        w_preset_1.onTrue( // Preset to pick up cube
            Commands.sequence(
                new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)),
                new InstantCommand(() -> s_Wrist.setRotation(Constants.wristCubePickup))
            )
        );

        w_preset_2.onTrue( // Preset to score mid cube
            Commands.sequence(
                new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorMidScore)),
                new WaitCommand(.25),
                new InstantCommand(() -> s_Wrist.setRotation(Constants.wristMidCubeScore))
            )
        );

        operator_stowButton.onTrue(new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)));
        operator_stowButton.onTrue(new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT)));

        driver_stowButton2.onTrue(new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)));
        driver_stowButton2.onTrue(new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT)));

        driver_stowButton.onTrue(new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)));
        driver_stowButton.onTrue(new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT)));

        driver_AutoBalance.onTrue(new InstantCommand(() -> s_Swerve.autoBalance()));

        zeroGyro.onTrue(new InstantCommand(s_Swerve::zeroGyro));

        // xModeButton.whileTrue(new InstantCommand(()-> s_Swerve.setXMode()));

    }

    public void printValues(){
        SmartDashboard.putBoolean("Pov pressed", e_presButton_0.getAsBoolean());
        SmartDashboard.putNumber("Elevator Position", s_Elevator.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator Target", s_Elevator.getTarget());
        SmartDashboard.putBoolean("elevator limit triggered?", s_Elevator.elevatorSwitchTriggered());
        SmartDashboard.putNumber("Wrist Position", s_Wrist.getEncoder().getPosition());
        SmartDashboard.putNumber("Wrist Target", s_Wrist.getTarget());
        SmartDashboard.putBoolean("cube beam broken?: ", s_Intake.cubeBeamBroken());
        SmartDashboard.putBoolean("cone beam broken?", s_Intake.coneBeamBroken());
        SmartDashboard.putNumber("intake speed", s_Intake.getSpeed());
        SmartDashboard.putNumber("yaw", s_Swerve.gyro.getYaw());
        SmartDashboard.putNumber("pitch", s_Swerve.gyro.getPitch());
        SmartDashboard.putNumber("roll", s_Swerve.gyro.getRoll());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand(String autoPicked) {

        Constants.gyroOffset = s_Swerve.gyro.getPitch();

        // Code that defines which autonomous to run from the selection in Shuffle Board
        String autoSelect = autoPicked.toLowerCase();

        switch (autoSelect) {
            case "audience":
                pPlan = "1 cycle";
                break;
            case "center":
                pPlan = "GrabConeMobility";
                break;
            case "twogamep":
                pPlan = "TwoGameP";
                break;
            case "collectbalanceaud":
                pPlan = "collectbalanceaud";
                break;
            case "collectbalancescore":
                pPlan = "collectbalancescore";
                break;
            case "scoremobilityengage":
                pPlan = "scoremobilityengage";
                break;
            case "scoremobilityengagepickup":
                pPlan = "scoremobilityengagepickup";
                break;
            case "scoremobilitycollectcablecarrier":
                pPlan = "scoremobilitycollectcablecarrier";
        }


        // This will load the path selected in Smart Dashboard and generate it with a max acceleration and velocity as defined for each section of path

        List<PathPlannerTrajectory> oneCycle = PathPlanner.loadPathGroup("1 cycle",
            new PathConstraints(2.5, 2),
            new PathConstraints(1.5, 1),
            new PathConstraints(0.5, 1),
            new PathConstraints(1.5, 1),
            new PathConstraints(1.5, 1),
            new PathConstraints(2.5, 2),
            new PathConstraints(1.5, 1)
        );

        List<PathPlannerTrajectory> grabConeMobility = PathPlanner.loadPathGroup("GrabConeMobility",
            new PathConstraints(1, 1)
        );

        List<PathPlannerTrajectory> foo = PathPlanner.loadPathGroup("TwoGameP",
            new PathConstraints(2.5, 2)
        );

        List<PathPlannerTrajectory> collectBalanceAudience = PathPlanner.loadPathGroup("collectbalanceaud",
            new PathConstraints(2.5, 2)
        );

        List<PathPlannerTrajectory> scoreMobilityEngage = PathPlanner.loadPathGroup("scoremobilityengage",
            new PathConstraints(1, 1)
        );

        List<PathPlannerTrajectory> collectBalanceScore = PathPlanner.loadPathGroup("collectbalancescore",
            new PathConstraints(1.3, 2),
            new PathConstraints(2.5, 2)
        );

        List<PathPlannerTrajectory> scoreMobilityEngagePickup = PathPlanner.loadPathGroup("scoremobilityengagepickup",
            new PathConstraints(1.5, 1.5)
        );

        List<PathPlannerTrajectory> scoreMobilityCollect = PathPlanner.loadPathGroup("scoremobilitycollect",
                new PathConstraints(2, 2)
        );

        List<PathPlannerTrajectory> scoreMobilityCollectCableCarrier = PathPlanner.loadPathGroup("scoremobilitycollectcablecarrier",
                new PathConstraints(2, 2)
        );

        List<PathPlannerTrajectory> rotate = PathPlanner.loadPathGroup("New Path",
                new PathConstraints(2, 1)
        );

        // Run a command when markers are passed. If you add a "balance" marker in Path Planner, this will cause the robot to run s_Swerve.autoBalance()
        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put("intakeOff", new AutoIntake(s_Intake, true)); // usually used after pickup cone/cube
        eventMap.put("balance", new InstantCommand(s_Swerve::autoBalance)); // balance on charge station
        eventMap.put("elevatorUp", new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorHighScore)));

        eventMap.put(
            "stow",
            Commands.sequence(
                new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)),
                new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT))
            )
        );

        eventMap.put( // picks up cone but does NOT stow automatically
            "intakeCone",
            Commands.sequence(
                new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorConePickup)),
                new InstantCommand(() -> s_Wrist.setRotation(Constants.wristConePickup)),
                new AutoIntake(s_Intake, true, true)
            )
        );

        eventMap.put( // picks up cube but does NOT stow automatically
            "intakeCube",
            Commands.sequence(
                new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)),
                new InstantCommand(() -> s_Wrist.setRotation(Constants.wristCubePickup)),
                new AutoIntake(s_Intake, false, true)
            )
        );

        eventMap.put( // scores cone and stows automatically
            "scoreConeHigh",
            Commands.sequence(
                new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorHighScore)),
                new InstantCommand(() -> s_Wrist.setRotation(Constants.wristHighConeScore)),
                new WaitCommand(1.2),
                new AutoIntake(s_Intake, true, false),
                new WaitCommand(0.5),
                new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)),
                new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT))
            )
        );

        eventMap.put( // scores cube and stows automatically
            "scoreCubeHigh",
            Commands.sequence(
                new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorHighScore)),
                new InstantCommand(() -> s_Wrist.setRotation(Constants.wristHighCubeScore)),
                new WaitCommand(1.2), //TODO could be slower
                new PrintCommand("setting intake"),
                new AutoIntake(s_Intake, false, false),
                new WaitCommand(0.5),
                new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)),
                new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT))
            )
        );

        // This defines swerve drive for autonomous path following
        // TODO: TUNE PID Constants
        SwerveAutoBuilder swerveControllerCommand = new SwerveAutoBuilder(
            s_Swerve::getPose,
            s_Swerve::resetOdometry,
            Constants.Swerve.swerveKinematics,
            new PIDConstants(1.6, 0.0, 0.0022), //translation Default: P: 1.5 I: 0.015 D: 0.01
            new PIDConstants(0.15, 0.075, 0.0), //rotation Default: P: 0.2 I: 0.04 D: 0.02
            s_Swerve::setModuleStates,
            eventMap,
            true,
            s_Swerve
        );

        switch (pPlan) {
            case "OneCycle":
                autoCode = Commands.sequence(
                    swerveControllerCommand.fullAuto(oneCycle.get(0)),
                    swerveControllerCommand.fullAuto(oneCycle.get(1)),
                    swerveControllerCommand.fullAuto(oneCycle.get(2)),
                    swerveControllerCommand.fullAuto(oneCycle.get(3)),
                    swerveControllerCommand.fullAuto(oneCycle.get(4)),
                    swerveControllerCommand.fullAuto(oneCycle.get(5)),
                    swerveControllerCommand.fullAuto(oneCycle.get(6))
                );
                break;
            case "GrabConeMobility":
                autoCode = swerveControllerCommand.fullAuto(grabConeMobility.get(0));
                break;
            case "TwoGameP":
                autoCode = swerveControllerCommand.fullAuto(foo.get(0));
                break;
            case "collectbalanceaud":
                autoCode = swerveControllerCommand.fullAuto(collectBalanceAudience.get(0));
                break;
            case "collectbalancescore":
                autoCode = Commands.sequence(
                    swerveControllerCommand.fullAuto(collectBalanceScore.get(0)),
                    swerveControllerCommand.fullAuto(collectBalanceScore.get(1))
                );
                break;
            case "scoremobilityengage":
                autoCode = swerveControllerCommand.fullAuto(scoreMobilityEngage.get(0));
                break;
            case "scoremobilityengagepickup":
                autoCode = swerveControllerCommand.fullAuto(scoreMobilityEngagePickup.get(0));
                break;
            case "scoremobilitycollect":
                autoCode = swerveControllerCommand.fullAuto(scoreMobilityCollect.get(0));
                break;
            case "scoremobilitycollectcablecarrier":
                autoCode = swerveControllerCommand.fullAuto(scoreMobilityCollectCableCarrier.get(0));
            default:
                autoCode = new PrintCommand(pPlan);
                break;
        }

        new InstantCommand(() -> s_Swerve.resetOdometry(rotate.get(0).getInitialHolonomicPose()));

        return autoCode;
    }
}
