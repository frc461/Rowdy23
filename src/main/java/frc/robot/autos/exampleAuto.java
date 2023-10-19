package frc.robot.autos;
/*
//package frc.robot.autos;

 import frc.robot.Constants;
 import frc.robot.subsystems.Intake;
 import frc.robot.subsystems.Swerve;

 import java.util.ArrayList;
 import java.util.HashMap;
 import java.util.List;

 import com.pathplanner.lib.PathConstraints;
 import com.pathplanner.lib.PathPlanner;
 import com.pathplanner.lib.PathPlannerTrajectory;
 import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
 import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
 import com.pathplanner.lib.auto.PIDConstants;
 import com.pathplanner.lib.auto.SwerveAutoBuilder;
 import com.pathplanner.lib.commands.FollowPathWithEvents;

 import edu.wpi.first.math.controller.PIDController;
 import edu.wpi.first.math.controller.ProfiledPIDController;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Rotation2d;
 import edu.wpi.first.math.geometry.Translation2d;
 import edu.wpi.first.math.trajectory.Trajectory;
 import edu.wpi.first.math.trajectory.TrajectoryConfig;
 import edu.wpi.first.math.trajectory.TrajectoryGenerator;
 import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.CommandBase;
 import edu.wpi.first.wpilibj2.command.InstantCommand;
 import edu.wpi.first.wpilibj2.command.PrintCommand;
 import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
 import edu.wpi.first.wpilibj2.command.Subsystem;
 import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

 public class exampleAuto extends FollowPathWithEvents  {
    public exampleAuto(Swerve s_Swerve){
        
        // This loads the constants for autonomous speed.
        PathConstraints config = new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        
        // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("Troubleshooting9.3.2023", config);

        // This trajectory can then be passed to a path follower such as a PPSwerveControllerCommand
        // Or the path can be sampled at a given point in time for custom path following

        // Sample the state of the path at 1.2 seconds
        PathPlannerState exampleState = (PathPlannerState) examplePath.sample(1.2);

        // Print the velocity at the sampled time
        System.out.println(exampleState.velocityMetersPerSecond);
                
        
         // An example trajectory to follow.  All units in meters.
         Trajectory exampleTrajectory =
             TrajectoryGenerator.generateTrajectory(
                 // Start at the origin facing the +X direction
                 new Pose2d(0, 0, new Rotation2d(0)),
                 // Pass through these two interior waypoints, making an 's' curve path
                 List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                 // End 3 meters straight ahead of where we started, facing forward
                 new Pose2d(3, 0, new Rotation2d(0))  );
        

         var thetaController =
             new ProfiledPIDController(
                 Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
         thetaController.enableContinuousInput(-Math.PI, Math.PI);

         s_Swerve.resetOdometry(examplePath.getInitialPose());

         HashMap<String, Command> eventMap = new HashMap<>();
         eventMap.put("marker1", new PrintCommand("Passed marker 1"));

         SwerveControllerCommand swervecontrollercommand = new SwerveControllerCommand(
                 examplePath,
                 s_Swerve::getPose,
                 Constants.Swerve.swerveKinematics,
                 new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                 new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                 thetaController,
                 s_Swerve::setModuleStates,
                 s_Swerve);
        

         FollowPathWithEvents autoWithEvents = new FollowPathWithEvents(swervecontrollercommand, examplePath.getMarkers(), eventMap);
           

        new InstantCommand(() -> s_Swerve.resetOdometry(s_Swerve.getPose()));
        
     }
 }
 */