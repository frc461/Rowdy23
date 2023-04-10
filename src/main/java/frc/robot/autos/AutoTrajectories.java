package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants;

public class AutoTrajectories {

    private PathPlannerTrajectory leaveCommunity0, leaveCommunity1, leaveCommunity2,
                             scoreLeave0, scoreLeave1, scoreLeave2,
                             scoreTwice0, scoreTwice1, scoreTwice2,
                             leaveCharge0, leaveCharge1, leaveCharge2,
                             scoreCharge0, scoreCharge1, scoreCharge2,
                             scoreTwiceCharge0, scoreTwiceCharge2;


    private final PathConstraints constraints, slowConstraints;

    public AutoTrajectories() {
        constraints = new PathConstraints(Constants.Auto.KMAX_SPEED_METERS_PER_SECOND, Constants.Auto.KMAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        //Coach(God(in his words)) said turn to 10
        slowConstraints = new PathConstraints(1.6, Constants.Auto.KMAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }

    public PathPlannerTrajectory getLeaveCommunity0() {
        leaveCommunity0 = PathPlanner.loadPath("LeaveCommunity0", constraints);
        return leaveCommunity0;
    }

    public PathPlannerTrajectory getLeaveCommunity1() {
        leaveCommunity1 = PathPlanner.loadPath("LeaveCommunity1", slowConstraints);
        return leaveCommunity1;
    }

    public PathPlannerTrajectory getLeaveCommunity2() {
        leaveCommunity2 = PathPlanner.loadPath("LeaveCommunity2", constraints);
        return leaveCommunity2;
    }

    public PathPlannerTrajectory getScoreLeave0() {
        scoreLeave0 = PathPlanner.loadPath("ScoreLeave0", constraints);
        return scoreLeave0;
    }

    public PathPlannerTrajectory getScoreLeave1(){
        scoreLeave1 = PathPlanner.loadPath("ScoreLeave1", slowConstraints);
        return scoreLeave1;
    }

    public PathPlannerTrajectory getScoreLeave2(){
        scoreLeave2 = PathPlanner.loadPath("ScoreLeave2", constraints);
        return scoreLeave2;
    }

    public PathPlannerTrajectory getScoreTwice0(){
        scoreTwice0 = PathPlanner.loadPath("ScoreTwo0", constraints);
        return scoreTwice0;
    }

    public PathPlannerTrajectory getScoreTwice1(){
        scoreTwice1 = PathPlanner.loadPath("ScoreTwo1", slowConstraints);
        return scoreTwice1;
    }

    public PathPlannerTrajectory getScoreTwice2(){
        scoreTwice2 = PathPlanner.loadPath("ScoreTwo2", constraints);
        return scoreTwice2;
    }

    public PathPlannerTrajectory getLeaveCharge0(){
        leaveCharge0 = PathPlanner.loadPath("LeaveCharge0", constraints);
        return leaveCharge0;
    }

    public PathPlannerTrajectory getLeaveCharge1(){
        leaveCharge1 = PathPlanner.loadPath("LeaveCharge1", slowConstraints);
        return leaveCharge1;
    }

    public PathPlannerTrajectory getLeaveCharge2(){
        leaveCharge2 = PathPlanner.loadPath("LeaveCharge2", constraints);
        return leaveCharge2;
    }

    public PathPlannerTrajectory getScoreCharge0(){
        scoreCharge0 = PathPlanner.loadPath("ScoreCharge0", constraints);
        return scoreCharge0;
    }

    public PathPlannerTrajectory getScoreCharge1(){
        scoreCharge1 = PathPlanner.loadPath("ScoreCharge1", slowConstraints);
        return scoreCharge1;
    }

    public PathPlannerTrajectory getScoreCharge2(){
        scoreCharge2 = PathPlanner.loadPath("ScoreCharge2", constraints);
        return scoreCharge2;
    }

    public PathPlannerTrajectory getScoreTwiceCharge0(){
        scoreTwiceCharge0 = PathPlanner.loadPath("ScoreTwoCharge0", constraints);
        return scoreTwiceCharge0;
    }

    public PathPlannerTrajectory getScoreTwiceCharge2(){
        scoreTwiceCharge2 = PathPlanner.loadPath("ScoreTwoCharge2", constraints);
        return scoreTwiceCharge2;
    }

    public PathPlannerTrajectory getScoreGrabCharge(){
        scoreTwiceCharge2 = PathPlanner.loadPath("ScoreGrabCharge", slowConstraints);
        return scoreTwiceCharge2;
    }
}