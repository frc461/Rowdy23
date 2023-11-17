package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

//plenty of newlines here to piss off beck :)


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Subscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;

public class Limelight extends SubsystemBase{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    DoubleArraySubscriber ySub;
    private double botPose[];
    public double botPoseX;
    public double botPoseZ;
    public int updates;

    public Limelight() {
        ySub = table.getDoubleArrayTopic("botpose_targetspace").subscribe(new double[6]);
        botPose = new double[6];
        botPoseX = 0.0;
        botPoseZ = 0.0;
        updates = 0;
    }

    public void refreshValues(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        botPose = ySub.get(new double[6]);
        botPoseX = botPose[0]; //X+ is to the right if you are looking at the tag
        botPoseZ = botPose[2]; //Z+ is perpendicular to the plane of the tag (Z+ is away from tag on data side, Z- is away on non data side)
        updates++;
        System.out.println("refresh");
    }

    public double getRX(){
        refreshValues();
        return botPose[0];
    }

    public double getRY(){
        refreshValues();
        return botPose[1];
    }

    public double getRZ(){
        refreshValues();
        return botPose[2];
    }

    public double getRoll(){
        refreshValues();
        return botPose[3];
    }

    public double getPitch(){
        refreshValues();
        return botPose[4];
    }

    public double getYaw(){
        refreshValues();
        return botPose[5];
    }

    public Pose3d getPose() {
        return new Pose3d(new Translation3d(getRX(), getRY(), getRZ()), new Rotation3d(getRoll(), getPitch(), getYaw()));
    }

    public Trajectory testTraj(Rotation2d yaw){
        TrajectoryConfig config = new TrajectoryConfig(0.5, 0.5);
        
        return TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(botPoseZ, botPoseX, Rotation2d.fromDegrees(botPose[4])),
                new Pose2d(.45, 0, Rotation2d.fromDegrees(0))
            ),
            config
        ); //theoretically this path goes to (0,0,0), the pos of the tag, from the robot's location
    }

    //Dont delete this eudard
    // public Trajectory testTraj(Rotation2d yaw){
    //     TrajectoryConfig config = new TrajectoryConfig(0.5, 0.5);
    //     var interiorWaypoints = new ArrayList<Translation2d>();
    //     interiorWaypoints.add(new Translation2d(-2,0));

    //     return TrajectoryGenerator.generateTrajectory(
    //     new Pose2d(botPoseZ, botPoseX, Rotation2d.fromDegrees(botPose[4])),
    //     interiorWaypoints,
    //     new Pose2d(0.65, 0, Rotation2d.fromDegrees(10)),
    //     config
    //     ); //theoretically this path goes to (0,0,0), the pos of the tag, from the robot's location
    // }

    

    //FollowPathWithEvents follower = new FollowPathWithEvents(null, null, null);    

    /* how to go to apriltag:
     * find tag (duh)
     * find position of robot relative to tag
     * generate trajectory from current position to tag (how?)
     * use trajectoryfollower to command the swerve to follow the path
    */

}