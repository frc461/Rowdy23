package frc.robot;

import java.time.Instant;
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
    private final Wrist s_Wrist = new Wrist(s_Elevator.getEncoder());

    private String pPlan = null;
    public double intakeVec = 0;

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


    private final POVButton w_preset_0 = new POVButton(operator, 0);
    private final POVButton w_preset_1 = new POVButton(operator, 90);
    private final POVButton w_preset_2 = new POVButton(operator, 180);
    private final POVButton operator_stowButton = new POVButton(operator, 270);


    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driver_stowButton2 = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final POVButton driver_stowButton = new POVButton(operator, 270);

    /* Variables */
    boolean driveStatus = false;
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        s_Elevator.setDefaultCommand(new TeleopElevator(s_Elevator, () -> -operator.getRawAxis(elevatorAxis)));

        s_Wrist.setDefaultCommand(new TeleopWrist(s_Wrist, () -> -operator.getRawAxis(wristAxis)));

        s_Intake.setDefaultCommand(new TeleopIntake(s_Intake, operator));
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
        w_preset_1.onTrue(new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_MID_LIMIT)));
        w_preset_2.onTrue(new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_LOWER_LIMIT)));
        
        e_presButton_0.onTrue(new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorTop)));
        e_presButton_1.onTrue(new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorMid)));
        e_presButton_2.onTrue(new InstantCommand(() -> s_Elevator.setHeight(6.52)));
        e_presButton_2.onTrue(new InstantCommand(() -> s_Wrist.setRotation(.485)));


        operator_stowButton.onTrue(new InstantCommand(() -> s_Elevator.setHeight(0)));
        operator_stowButton.onTrue(new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT)));

        driver_stowButton.onTrue(new InstantCommand(() -> s_Elevator.setHeight(0)));
        driver_stowButton.onTrue(new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT)));

        driver_stowButton2.onTrue(new InstantCommand(() -> s_Elevator.setHeight(0)));
        driver_stowButton2.onTrue(new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT)));
        
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        
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

        if(autoSelect.equals("center")){
          pPlan = "finalCC_DE";
        }
        else if(autoSelect.equals("scoring")){
          pPlan = "finalFR";
        }
        else if(autoSelect.equals("audience")){
          pPlan = "finalFL";
        }
        else if(autoSelect.equals("rnd")){
          pPlan = "rnd";
        }
        else if(autoSelect.equals("centerbonus")) {
          pPlan = "finalCC_DEM";
        }
        else if (autoSelect.equals("1 cycle")) {
          pPlan = "1 cycle";
        }
        else if (autoSelect.equals("2 cycles")) {
          pPlan = "2cycles";
        }
        else if (autoSelect.equals("dockBalance")) {
          pPlan = "no path";
        }
        else {
          pPlan = "noAuto";
        }

        // This will load the path selected in Smart Dashboard and generate it with a max acceleration and velocity as defined for each section of path
        List<PathPlannerTrajectory> rowdyPath = PathPlanner.loadPathGroup(pPlan, 
        new PathConstraints(1.5,2),   // Speed and acceleration for first path
        new PathConstraints(1, 0.5)   // Speed and acceleration for second path
        );

        List<PathPlannerTrajectory> clockwise180 = PathPlanner.loadPathGroup("Clockwise180", 
        new PathConstraints(.75,2)
        );

        List<PathPlannerTrajectory> counterclockwise180 = PathPlanner.loadPathGroup("Counterclockwise180", 
        new PathConstraints(.75,2)
        );

        List<PathPlannerTrajectory> oneCycleMove = PathPlanner.loadPathGroup("move3.2Meters", 
        new PathConstraints(4,2)
        );

        List<PathPlannerTrajectory> oneCycleMoveBack = PathPlanner.loadPathGroup("move-3.2Meters",
        new PathConstraints(4, 2)
        );

        // List<PathPlannerTrajectory> oneCycleCollect = PathPlanner.loadPathGroup("move.25Meters",
        // new PathConstraints(1.5, 2)
        // );

        List<PathPlannerTrajectory> oneCycleScore = PathPlanner.loadPathGroup("move-.25Meters",
        new PathConstraints(1.5, 2)
        );

        List<PathPlannerTrajectory> oneCycleMoveUp = PathPlanner.loadPathGroup("moveUp",
        new PathConstraints(2.5, 2)
        );
        
        // Run a command when markers are passed. If you add a "balance" marker in Path Planner, this will cause the robot to run s_Swerve.autoBalance()
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("start1", new PrintCommand("at start"));
        eventMap.put("mid", new PrintCommand("halfway done"));
        eventMap.put("elevatorUp", new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorTop)));
        eventMap.put("balance", new InstantCommand(() -> s_Swerve.autoBalance()));
        
        // This defines swerve drive for autonomous path following
        // TODO: TUNE PID Constants
        SwerveAutoBuilder swervecontrollercommand = new SwerveAutoBuilder(
          s_Swerve::getPose,
          s_Swerve::resetOdometry,
          Constants.Swerve.swerveKinematics,
          new PIDConstants(1.5, 0.015, 0.01),
          new PIDConstants(0.2, 0.35, 0),
          s_Swerve::setModuleStates,
          eventMap,
          true,
          s_Swerve
        );

        autoCode = Commands.sequence(
          // Score a cube command
          new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorTop)),
          new WaitCommand(1.2),
          new AutoIntakeCmd(s_Intake, -1),
          new WaitCommand(0.5),
          new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorLow)),
          swervecontrollercommand.fullAuto(oneCycleMove.get(0)),
          new PrintCommand("path 1 complete"),
          swervecontrollercommand.fullAuto(clockwise180.get(0)),
          new PrintCommand("path 2 complete"),
          // Pickup cone command
          swervecontrollercommand.fullAuto(oneCycleScore.get(0)),
          new PrintCommand("path 3 complete"),
          swervecontrollercommand.fullAuto(counterclockwise180.get(0)),
          new PrintCommand("path 4 complete"),
          // run another path
          swervecontrollercommand.fullAuto(oneCycleMoveUp.get(0)),
          swervecontrollercommand.fullAuto(oneCycleMoveBack.get(0)),
          new PrintCommand("path 5 complete"),
          // score cone command
          swervecontrollercommand.fullAuto(oneCycleMoveUp.get(0)),
          swervecontrollercommand.fullAuto(oneCycleScore.get(0)),
          new PrintCommand("path 6 complete")
          // run another path

        );


        //Command fullAuto = swervecontrollercommand.fullAuto(rowdyPath);

        new InstantCommand(() -> s_Swerve.resetOdometry(s_Swerve.getPose()));
              
        // SwerveModuleState[] xMode = {
        //     new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        //     new SwerveModuleState(0, Rotation2d.fromDegrees(45+90)),
        //     new SwerveModuleState(0, Rotation2d.fromDegrees(45+90+90)),
        //     new SwerveModuleState(0, Rotation2d.fromDegrees(45+90+90+90))
        // };

        // s_Swerve.setModuleStates(xMode);
        

      return autoCode;
    }
}
