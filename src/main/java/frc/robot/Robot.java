// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import org.json.simple.parser.ParseException;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Auto.Auto;
import frc.robot.Subsystems.*;
import frc.robot.Utilities.ShuffleboardTable;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    //Constants

    //private final SendableChooser<Paths> AUTO_CHOOSER = new SendableChooser<>();
    public Teleop teleop;
    public Auto auto;

    private InstantCommand lowerSpeed;
    private InstantCommand heightenSpeed;

    //Deadzone constants

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {

        lowerSpeed = new InstantCommand(() -> {
            Constants.MAX_SPEED = 0.0;
            Constants.MAX_ANGULAR_SPEED = 1.0;
        });
        lowerSpeed.setName("Lower Speed");

        //SmartDashboard.putData(lowerSpeed);

        heightenSpeed = new InstantCommand(() -> {
            Constants.MAX_SPEED = 4.75;
            Constants.MAX_ANGULAR_SPEED = 8.0 * Math.PI;
        });
        heightenSpeed.setName("Heighten Speed");

        //SmartDashboard.putData(heightenSpeed);

        try {
            Constants.TUNING_TABLE = ShuffleboardTable.fromJSON("tuning_layout.json");
        } catch (IOException | ParseException e1) {
            
        }
        Subsystems.initSubsystems();
        auto = new Auto();
        teleop = new Teleop();
        // Paths[] paths = Paths.values();
        // for(int pathInd = 0; pathInd < paths.length; pathInd++){
        //     AUTO_CHOOSER.addOption(paths[pathInd].name(), paths[pathInd]);
        // }
        // AUTO_CHOOSER.setDefaultOption("OFF_TARMAC", Paths.OFF_TARMAC);
        // //SmartDashboard.putData("Auto choices", AUTO_CHOOSER);
        
        //Subsystems.getShooter().putShooterDataToDashboard();

        //Limelight.setLEDMode(LEDMode.OFF);

        Constants.TUNING_TABLE.putNumber("Theta A", Constants.THETA_A);
        Constants.TUNING_TABLE.putNumber("Theta B", Constants.THETA_B);
        Constants.TUNING_TABLE.putNumber("Theta C", Constants.THETA_C);

        Constants.TUNING_TABLE.putNumber("Omega A", Constants.OMEGA_A);
        Constants.TUNING_TABLE.putNumber("Omega B", Constants.OMEGA_B);
        Constants.TUNING_TABLE.putNumber("Omega C", Constants.OMEGA_C);

        //Subsystems.getIntake().beamBreaksToSmart();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {

        if(lowerSpeed.isScheduled()){
            lowerSpeed.execute();
            lowerSpeed.cancel();
        }

        if(heightenSpeed.isScheduled()){
            heightenSpeed.execute();
            heightenSpeed.cancel();
        }
        // Subsystems.getDriveTrain().putCANCodersToSmartDashboard();

        // Subsystems.getShooter().putShooterDataToDashboard();
        // Subsystems.getIntake().beamBreaksToSmart();

    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different
     * autonomous modes using the dashboard. The sendable chooser code works with
     * the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard (if you hate yourself!), remove all of the
     * chooser code and
     * uncomment the getString line to get the auto name from the text box below the
     * Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure
     * below with additional strings. If using the SendableChooser make sure to add
     * them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {

        //auto.setPath(AUTO_CHOOSER.getSelected());
        //auto.initPath();
        //teleop.autod();
        // PID Controllers
        auto.init461();
    }

    /** This function is called periodically during autonomous. */
    
    @Override
    public void autonomousPeriodic() {
        
        auto.run461();


        //PathPlannerTrajectory testPath = PathPlanner.loadPath("NewPath", 4, 3);
        //PathPlannerState testState = (PathPlannerState) testPath.sample(1.2);
        //System.out.println(testState.velocityMetersPerSecond);
        //Subsystems.getDriveTrain().drive(-speeds.vyMetersPerSecond, -speeds.vxMetersPerSecond, -speeds.omegaRadiansPerSecond, false);
        // Pose2d currentOdomPos = Subsystems.getDriveTrain().getOdometryPoseMeters();
        // PathPlannerState goal = (PathPlannerState) paths[path.index].sample(currentTime);
        // ChassisSpeeds speeds = controller.calculate(currentOdomPos, goal,
        //         goal.holonomicRotation);
        // if(!stopped){
        //     Subsystems.getDriveTrain().drive(-speeds.vyMetersPerSecond, -speeds.vxMetersPerSecond, -speeds.omegaRadiansPerSecond, false);
        // }
        
        
        
        // while(Subsystems.getDriveTrain().getFLEncoder() < 100){
        //     Subsystems.getDriveTrain().drive(3, 0, 0, true);
        // }
        // while(Subsystems.getDriveTrain().getFLEncoder() < 200){
        //     Subsystems.getDriveTrain().drive(0, 5, 0, true);
        // }
        // while(Subsystems.getDriveTrain().getFLEncoder() < 300){
        //     Subsystems.getDriveTrain().drive(-4, 0, 0, true);
        // }
        // while(Subsystems.getDriveTrain().getFLEncoder() < 400){
        //     Subsystems.getDriveTrain().drive(0, -1, 0, true);
        // }

        // Pose2d currentOdomPos = Subsystems.getDriveTrain().getOdometryPoseMeters();
        // PathPlannerTrajectory testPath = PathPlanner.loadPath("New New Path", 4, 3);
        // PathPlannerState testState = (PathPlannerState) testPath.sample(1.2);
        
        // ChassisSpeeds speeds = controller.calculate(currentOdomPos, testState,
        //         testState.holonomicRotation);
        // Subsystems.getDriveTrain().drive(-speeds.vyMetersPerSecond, -speeds.vxMetersPerSecond, -speeds.omegaRadiansPerSecond, false);
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        //auto.stopAuto();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        teleop.run();

    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}