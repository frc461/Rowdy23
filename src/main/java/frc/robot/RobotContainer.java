package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import frc.robot.commands.AutoIntakeCmd;

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
        e_presButton_2.onTrue(new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorLow)));

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
    }

    // private Command stow(){
    //      s_Elevator.setElevatorPreset(Constants.elevatorStowPos);
    //      s_Intake.setRotation(Constants.wristStowPos);
    // }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(String autoPicked) {
        // An ExampleCommand will run in autonomous
        //return new exampleAuto(s_Swerve);
        
        System.out.println(autoPicked);

        PathConstraints config2 = new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        FollowPathWithEvents autoWithEvents = null;
        
        
        String autoSelect = autoPicked.toLowerCase();
        System.out.println(autoSelect);

        if(autoSelect.equals("center")){
          pPlan = "finalCC_DE";
        }
        else  if(autoSelect.equals("scoring")){
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
        else{
          pPlan = "noAuto";
        }
        

        //List<PathPoint> points;
        // points = new ArrayList<PathPoint>();
        // points.add( new PathPoint(new Translation2d(2.7, 0), Rotation2d.fromDegrees(0)));
            //System.out.println("Starting command process" + Timer.getMatchTime());
            
            SequentialCommandGroup startAuto = new SequentialCommandGroup(
            
                // Set Elevator to top height
                new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorTop)),

                // Wait 1.25 seconds before next command
                new WaitCommand(1),
                
                // Turn intake on to eject cube using AutoIntakeCmd
                new AutoIntakeCmd(s_Intake, -1),

                // Set Elevator to low height
                new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)),

                // Wait 1.25 seconds before next command
                new WaitCommand(1.25)

                // Make the robot drive
                //new AutoDriveCmd(s_Swerve, pPlan)

                );
        
        startAuto.schedule();
        startAuto.execute();

     
        SwerveModuleState[] xMode = {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45+90)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45+90+90)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45+90+90+90))
        };

        s_Swerve.setModuleStates(xMode);
        
 
            PathPlannerTrajectory autoTrajectory = PathPlanner.loadPath(pPlan, config2);
             for (int i = 0; i < autoTrajectory.getMarkers().size(); i++) {
            }
            

            System.out.println("While driving: " + Timer.getMatchTime());

            var thetaController =
                new ProfiledPIDController(
                    Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
            s_Swerve.resetOdometry(autoTrajectory.getInitialHolonomicPose());
    
            HashMap<String, Command> eventMap = new HashMap<>();
            eventMap.put("xmode", new InstantCommand(()->s_Swerve.setModuleStates(xMode)));   
    
            SwerveControllerCommand swervecontrollercommand = new SwerveControllerCommand(
                    autoTrajectory,
                    s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);
            
    
            autoWithEvents = new FollowPathWithEvents(swervecontrollercommand, autoTrajectory.getMarkers(), eventMap);
          
           s_Swerve.setModuleStates(xMode);   

            return autoWithEvents;
    
             
        
           
    }

}
