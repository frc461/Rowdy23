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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;

public class Limelight extends SubsystemBase{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private double botpose[];
    private double botposefromtag[];
    public double botPose2d[];

    public Limelight() {
        botPose2d = new double[2];
    }

    public void refreshValues(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        botpose = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        System.out.println("refresh");
    }

    public double getRX(){
        refreshValues();
        return botpose[0];
    }

    public double getRY(){
        refreshValues();
        return botpose[1];
    }

    public double getRZ(){
        refreshValues();
        return botpose[2];
    }

    public double getRoll(){
        refreshValues();
        return botpose[3];
    }

    public double getPitch(){
        refreshValues();
        return botpose[4];
    }

    public double getYaw(){
        refreshValues();
        return botpose[5];
    }

    public Pose3d getPose() {
        return new Pose3d(new Translation3d(getRX(), getRY(), getRZ()), new Rotation3d(getRoll(), getPitch(), getYaw()));
    }

    public Trajectory testTraj(Rotation2d yaw){
        botposefromtag = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        botPose2d[0] = botposefromtag[0]; //X+ is to the right if you are looking at the tag
        botPose2d[1] = botposefromtag[2]; //Z+ is perpendicular to the plane of the tag (Z+ is away from tag on data side, Z- is away on non data side)

        
        TrajectoryConfig config = new TrajectoryConfig(0.5, 0.5);
        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(0.3, 0.3)); //you can add waypoints like this
        return TrajectoryGenerator.generateTrajectory(
        new Pose2d(-botPose2d[0], 
        botPose2d[1], yaw), 
        interiorWaypoints, 
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
        config); //theoretically this path goes to (0,0,0), the pos of the tag, from the robot's location
    }

    

    //FollowPathWithEvents follower = new FollowPathWithEvents(null, null, null);    

    /* how to go to apriltag:
     * find tag (duh)
     * find position of robot relative to tag
     * generate trajectory from current position to tag (how?)
     * use trajectoryfollower to command the swerve to follow the path
    */

}

