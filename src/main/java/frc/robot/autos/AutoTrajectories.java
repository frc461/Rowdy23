package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants;

public class AutoTrajectories {

    private PathPlannerTrajectory defaultAuto, grabConeMobility, collectBalanceAudience, scoreMobilityEngage, 
    collectBalanceScore, scoreMobilityEngagePickup, scoremobilitycollect, scoremobilitycollectcablecarrier;


    private final PathConstraints constraints, slowConstraints;

    public AutoTrajectories() {
        constraints = new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAccel);
        //Coach(God(in his words)) said turn to 10
        slowConstraints = new PathConstraints(Constants.AutoConstants.slowVel, Constants.AutoConstants.slowAccel);
    }

    public PathPlannerTrajectory defaultAuto() {
        defaultAuto = PathPlanner.loadPath("defaultAuto", constraints);
        return defaultAuto;
    }

    public PathPlannerTrajectory grabConeMobility() {
        grabConeMobility = PathPlanner.loadPath("GrabConeMobility", constraints);
        return grabConeMobility;
    }

    public PathPlannerTrajectory collectBalanceScore() {
        collectBalanceScore = PathPlanner.loadPath("collectbalancescore", slowConstraints);
        return collectBalanceScore;
    }

    public PathPlannerTrajectory collectBalanceAudience() {
        collectBalanceAudience = PathPlanner.loadPath("collectbalanceaud", constraints);
        return collectBalanceAudience;
    }

    public PathPlannerTrajectory scoreMobilityEngage() {
        scoreMobilityEngage = PathPlanner.loadPath("scoremobilityengage", constraints);
        return scoreMobilityEngage;
    }

    public PathPlannerTrajectory scoreMobilityEngagePickup(){
        scoreMobilityEngagePickup = PathPlanner.loadPath("scoremobilityengagepickup", slowConstraints);
        return scoreMobilityEngagePickup;
    }

    public PathPlannerTrajectory scoreMobilityCollect(){
        scoremobilitycollect = PathPlanner.loadPath("scoremobilitycollect", constraints);
        return scoremobilitycollect;
    }

    public PathPlannerTrajectory scoreMobilityCollectCableCarrier(){
        scoremobilitycollectcablecarrier = PathPlanner.loadPath("scoremobilitycollectcablecarrier", constraints);
        return scoremobilitycollectcablecarrier;
    }

}