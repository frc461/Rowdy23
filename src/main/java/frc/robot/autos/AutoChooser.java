
package frc.robot.autos;

import java.time.chrono.ThaiBuddhistEra;
import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class AutoChooser {
    private final AutoTrajectories trajectories;
    private final Swerve s_Swerve;
    private final Intake s_Intake;
    private final Wrist s_Wrist;
    private final Elevator s_Elevator;
    

    private final SendableChooser<AutonomousMode> m_chooser= new SendableChooser<>();
    private HashMap<String, Command> eventMap;
    private PIDController thetaController = new PIDController(0.05, 0.005, 0.009);


    public AutoChooser(AutoTrajectories trajectories, HashMap<String, Command> eventMap, Swerve s_Swerve, Intake s_Intake, Wrist s_Wrist, Elevator s_Elevator) {
        this.s_Swerve = s_Swerve;
        this.s_Elevator = s_Elevator;
        this.s_Intake = s_Intake;
        this.s_Wrist = s_Wrist;
        this.eventMap = eventMap;
        this.trajectories = trajectories;
        

        m_chooser.setDefaultOption("Default Auto", AutonomousMode.kDefaultAuto);
        m_chooser.addOption("CenterAuto", AutonomousMode.kCenterAuto);
        m_chooser.addOption("Audience Side (1 cycle)", AutonomousMode.kAudienceAuto);
        m_chooser.addOption("Two Game P", AutonomousMode.kTwoGameP);
        m_chooser.addOption("Collect And Balance Audience Side", AutonomousMode.kCollectBalanceAud);
        m_chooser.addOption("Collect And Balance Scoring Table Side", AutonomousMode.kCollectBalanceScore);
        m_chooser.addOption("Score Mobility Engage", AutonomousMode.kScoreMobilityEngage);
        m_chooser.addOption("score mobility engage pickup", AutonomousMode.kScoremobilityengagepickup);
        m_chooser.addOption("non cc score mobility collect", AutonomousMode.kScoremobilitycollect);
        m_chooser.addOption("cc side score mobility collect", AutonomousMode.kScoremobilitycollectcablecarrier);
        m_chooser.addOption("two piece", AutonomousMode.kTwoPiece);
        m_chooser.addOption("three low", AutonomousMode.kThreePiece);

    }

    public SendableChooser<AutonomousMode> getAutoChooser() {
        return m_chooser;
    }

    public PIDController getPIDController() {
        return thetaController;
    }

    public Command defaultAuto() {
        var swerveCommand = createControllerCommand(trajectories.defaultAuto());
        
        FollowPathWithEvents followCommand = new FollowPathWithEvents(
        swerveCommand, 
        trajectories.defaultAuto().getMarkers(), 
        eventMap);

        SequentialCommandGroup command = new SequentialCommandGroup();
        //command.addCommands( eventMap.get("scoreCubeHigh"));
        command.addCommands(
            new SequentialCommandGroup(eventMap.get("scoreCubeHigh")),
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectories.defaultAuto().getInitialHolonomicPose())),
            new SequentialCommandGroup(followCommand)
        );
        return command;
    }

    public Command scoreMobilityCollect() {
        var swerveCommand = createControllerCommand(trajectories.scoreMobilityCollect());
        
        FollowPathWithEvents followCommand = new FollowPathWithEvents(
        swerveCommand, 
        trajectories.scoreMobilityCollect().getMarkers(), 
        eventMap);

        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
            // new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorHighScore)),
            // new InstantCommand(() -> s_Wrist.setRotation(Constants.wristHighCubeScore)),
            // new WaitCommand(1.2), //TODO could be slower
            // new PrintCommand("setting intake"),
            // new AutoIntakeCommand(s_Intake, -1, false),
            // new WaitCommand(0.5),
            // new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)),
            // new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT)),
            new SequentialCommandGroup(eventMap.get("scoreCubeHigh")),
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectories.scoreMobilityCollect().getInitialHolonomicPose())),
            new SequentialCommandGroup(followCommand)
        );
        return command;
    }

    public Command twoPiece() {
        var swerveCommand = createControllerCommand(trajectories.twoPiece());
        
        FollowPathWithEvents followCommand = new FollowPathWithEvents(
        swerveCommand, 
        trajectories.twoPiece().getMarkers(), 
        eventMap);

        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
            new SequentialCommandGroup(eventMap.get("scoreCubeHigh")),
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectories.twoPiece().getInitialHolonomicPose())),
            new SequentialCommandGroup(followCommand),
            new SequentialCommandGroup(eventMap.get("scoreConeHigh"))
        );
        return command;
    }

    public Command threePiece() {
        var swerveCommand = createControllerCommand(trajectories.threePiece());
        
        FollowPathWithEvents followCommand = new FollowPathWithEvents(
        swerveCommand, 
        trajectories.threePiece().getMarkers(), 
        eventMap);

        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
           // new SequentialCommandGroup(eventMap.get("scoreCubeHigh")),
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectories.threePiece().getInitialHolonomicPose())),
            new SequentialCommandGroup(followCommand),
            new SequentialCommandGroup(eventMap.get("coneOut"))
        );
        return command;
    }

    public PPSwerveControllerCommand createControllerCommand(PathPlannerTrajectory trajectory) {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        return new PPSwerveControllerCommand
        (trajectory, 
        s_Swerve::getPose,
        Constants.Swerve.swerveKinematics , 
        new PIDController(1, 0, 0), 
        new PIDController(1, 0, 0), 
        thetaController,
        s_Swerve::setModuleStates,
        false,
        s_Swerve
        );
    }

    public Command getCommand() {
        switch (m_chooser.getSelected()) {
            case kDefaultAuto :
            return defaultAuto();
            case kScoremobilitycollect :
            return scoreMobilityCollect();
            case kTwoPiece:
            return twoPiece();
            case kThreePiece:
            return threePiece();
        }
        return defaultAuto();
    }


    private enum AutonomousMode {
        kDefaultAuto, kAudienceAuto, kCenterAuto, kTwoGameP, kCollectBalanceAud,
        kCollectBalanceScore, kScoreMobilityEngage, kScoremobilityengagepickup,
        kScoremobilitycollect, kScoremobilitycollectcablecarrier, kTwoPiece, kThreePiece
    }

    

}