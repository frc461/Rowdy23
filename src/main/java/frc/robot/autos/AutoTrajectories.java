package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants;

public class AutoTrajectories {

    private PathPlannerTrajectory defaultAuto, grabConeMobility, collectBalanceAudience, scoreMobilityEngage, 
    collectBalanceScore, scoreMobilityEngagePickup, scoremobilitycollect, scoremobilitycollectcablecarrier, twoPiece,
    threePiece, twoCube, twoCubeCC, alternatePickup, threeLow;


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
        scoreMobilityEngage = PathPlanner.loadPath("scoremobilityengage", slowConstraints);
        return scoreMobilityEngage;
    }

    public PathPlannerTrajectory scoreMobilityEngagePickup(){
        scoreMobilityEngagePickup = PathPlanner.loadPath("scoremobilityengagepickup", slowConstraints);
        return scoreMobilityEngagePickup;
    }

    public PathPlannerTrajectory scoreMobilityCollect(){
        scoremobilitycollect = PathPlanner.loadPath("scoremobilitycollect", slowConstraints);
        return scoremobilitycollect;
    }

    public PathPlannerTrajectory scoreMobilityCollectCableCarrier(){
        scoremobilitycollectcablecarrier = PathPlanner.loadPath("scoremobilitycollectcablecarrier", slowConstraints);
        return scoremobilitycollectcablecarrier;
    }

    public PathPlannerTrajectory twoPiece(){
        twoPiece = PathPlanner.loadPath("TwoPiece", constraints);
        return twoPiece;
    }

    public PathPlannerTrajectory threeLow(){
        threeLow = PathPlanner.loadPath("ThreeLow", constraints);
        return threeLow;
    }
    public PathPlannerTrajectory twoCube(){
        twoCube = PathPlanner.loadPath("TwoCube", constraints);
        return twoCube;
    }
    
    public PathPlannerTrajectory twoCubeCC(){
        twoCubeCC = PathPlanner.loadPath("TwoCubeCC", constraints);
        return twoCubeCC;
    }

    public PathPlannerTrajectory alternatePickup(){
        alternatePickup = PathPlanner.loadPath("alternatePickup", constraints);
        return alternatePickup;
    }

    public PathPlannerTrajectory threePiece(){
        threePiece = PathPlanner.loadPath("ThreePiece", constraints);
        return threePiece;
    }
}