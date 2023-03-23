// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private static final String kDefaultAuto = "no auto";
  private static final String kAudienceAuto = "audience";
  private static final String kCenterAuto = "center";
  private static final String kTwoGameP = "TwoGameP";
  private static final String kCollectBalanceAud = "collectbalanceaud";
  private static final String kCollectBalanceScore = "collectbalancescore";
  private static final String kScoreMobilityEngage = "scoremobilityengage";
  private static final String kScoremobilityengagepickup = "scoremobilityengagepickup";
  private static final String kScoremobilitycollect = "scoremobilitycollect";
  private static final String kScoremobilitycollectcablecarrier = "scoremobilitycollectcablecarrier";


  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
   
    m_chooser.setDefaultOption("No Auto Selected", kDefaultAuto);
    m_chooser.addOption("CenterAuto", kCenterAuto);
    m_chooser.addOption("Audience Side (1 cycle)", kAudienceAuto);
    m_chooser.addOption("Two Game P", kTwoGameP);
    m_chooser.addOption("Collect And Balance Audience Side", kCollectBalanceAud);
    m_chooser.addOption("Collect And Balance Scoring Table Side", kCollectBalanceScore);
    m_chooser.addOption("Score Mobility Engage", kScoreMobilityEngage);
    m_chooser.addOption("score mobility engage pickup", kScoremobilityengagepickup);
    m_chooser.addOption("Side score mobility collect", kScoremobilitycollect);
    m_chooser.addOption("cable carrier side score mobility collect", kScoremobilitycollectcablecarrier);
    
    SmartDashboard.putData("Auto Choices", m_chooser);

    ctreConfigs = new CTREConfigs();
    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    PathPlannerServer.startServer(5811);

    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    m_robotContainer.printValues();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_chooser.getSelected());

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
