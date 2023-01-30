package frc.robot.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.*;

public class Auto {
    
    //The Paths enum is used to select our path
    //The name of the enum is what the file that holds the path data is called
    //the first parameter is an index. The index is what links to a few other things
    //the rest of the parameters are time based setpoints for shooting, intaking, and other things
    // public enum Paths {
    //     HANGAR_2_BALL(0,
    //         new Setpoint(-1.0, 1.0,
    //             () -> Subsystems.getPneumatics().intakeOut())
    //     ),
    //     TERMINAL_3_BALL(1,
    //     new Setpoint(-2.0, 2.0,
    //         () -> Setpoint.noop())
    //     // new Setpoint(0.55, 0.1, 
    //     //     () -> Subsystems.getPneumatics().intakeOut(), 
    //     //     () -> Subsystems.getShooter().setShooter(ShooterPositions.TARMAC)),
    //     // new Setpoint(0.75, 4.5,
    //     //     () -> Subsystems.getIntake().runIntake(0.75),
    //     //     () -> Subsystems.getIntake().stopIntake()),
    //     // new Setpoint(5.25, 1.0,
    //     //     () -> Subsystems.getShooter().shoot()
    //     // )),
    //     ),
    //     TERMINAL_2_BALL(2,
    //     new Setpoint(-0.25, 0.25,
    //         () -> Subsystems.getShooter().shoot())
    //     // new Setpoint(0.0, 0.1,
    //     //     () -> Subsystems.getShooter().stopShooterAndFeed()),
    //     // new Setpoint(1.0, 3.75, 
    //     //     () -> Subsystems.getShooter().stopShooterAndFeed(),
    //     //     () -> Subsystems.getIntake().runIntake(0.8)),
    //     // new Setpoint(4.8, 0.1,
    //     //     () -> Setpoint.noop(),
    //     //     () -> Subsystems.getIntake().stopIntake(),
    //     //     () -> Setpoint.noop()),
    //     // new Setpoint(6.0, 3.0,
    //     //     () -> Subsystems.getShooter().shoot()
    //     // )),
    //     ),
    //     OFF_TARMAC(3),
    //     TUNE_PATH(4),
    //     TERMINAL_5_BALL(5,
    //     new Setpoint(-1.75, 1.75,
    //         () -> Subsystems.getShooter().setShooter(ShooterPositions.FENDER_HIGH), 
    //         () -> Subsystems.getShooter().shoot(), 
    //         () -> Subsystems.getShooter().stopShooter()),
    //     new Setpoint(0.1, 0.1, 
    //         () -> Subsystems.getPneumatics().intakeOut(), 
    //         () -> Setpoint.noop(), 
    //         () -> Subsystems.getIntake().runIntake(0.75, false, false, false)),
    //     new Setpoint(1.0, 1.75, 
    //         () -> Subsystems.getShooter().setShooter(ShooterPositions.TARMAC), 
    //         () -> Subsystems.getIntake().runIntake(0.75, false, false, false), 
    //         () -> Subsystems.getIntake().runIntake(0.75, false, false, false)),
    //     new Setpoint(2.75, 1.25, 
    //         () -> Subsystems.getIntake().runIntake(0.4, false, false, false),  
    //         () ->Subsystems.getShooter().revShooter(), 
    //         () ->Subsystems.getIntake().stopIntake()),
    //     new Setpoint(5.5, 0.5, 
    //         () -> Subsystems.getIntake().runIntake(0.75, false, false, true), 
    //         () -> Subsystems.getShooter().shoot()),
    //     new Setpoint(5.0, 4.0, 
    //         () -> Subsystems.getShooter().revShooter(), 
    //         () -> Subsystems.getShooter().shoot(), 
    //         () -> Subsystems.getShooter().stopShooter())/*,
    //     new Setpoint(8.5, 2.0, 
    //         () -> Subsystems.getShooter().setShooter(ShooterPositions.TARMAC), 
    //         () -> Subsystems.getIntake().runIntake(0.75), 
    //         () -> Subsystems.getIntake().runIntake(0.0)),
    //     new Setpoint(10.5, 1.0, 
    //         () -> Setpoint.noop(), 
    //         () -> Subsystems.getShooter().revShooter(), 
    //         () -> Setpoint.noop()),
    //     new Setpoint(12.0, 2.0, 
    //         () -> Setpoint.noop(), 
    //         () -> Subsystems.getShooter().shoot(), 
    //         () -> Subsystems.getShooter().stopShooterAndFeed())*/
    //     ),
    //     HANGAR_2_BALL_2(6);

    //     final int index;
    //     final Setpoint[] setpoints;
    //     Paths(int index, Setpoint... setpoints) {
    //         this.index = index;
    //         this.setpoints = setpoints;
    //     }
    // }

    //private PathPlannerTrajectory[] paths = new PathPlannerTrajectory[Paths.values().length];
    //private Pose2d[] startPoints = new Pose2d[Paths.values().length];
    //private boolean fiveBall = false;
    private static Timer timer = new Timer();

    // PID Controllers
    public PIDController xPid = new PIDController(2.0, 0.01, 0);
    public PIDController yPid = new PIDController(2.0, 0.01, 0);
    public ProfiledPIDController thetaPid = new ProfiledPIDController(4.5, 0, 0,
    new TrapezoidProfile.Constraints(Constants.MAX_ANGULAR_SPEED, Constants.MAX_ANGULAR_ACC));

    private SwerveAutoController controller = new SwerveAutoController(
            xPid,
            yPid,
            thetaPid
    );

    //private Paths path;

    public Auto() {
        // Paths[] paths = Paths.values();
        
        // for(int i = 0; i < paths.length; i++){
        //    int index = paths[i].index;
        //    String name = paths[i].name();
        //    this.paths[index] = PathPlanner.loadPath(name, Constants.MAX_SPEED, Constants.MAX_ACC);
        //    this.startPoints[index] = this.paths[index].getInitialState().poseMeters;
        // }

        thetaPid.enableContinuousInput(-180, 180);
    }

    /**
     * Sets path equal to path inputed
     * 
     * @param path
     */
    // public void setPath(Paths path) {
    //     this.path = path;
    // }

    /**
     * Initiates the path by setting the odometry to that of the path's start point,
     * and starting the time
     */
    
    //  public void initPath() {
    //     if(path.equals(Paths.TERMINAL_5_BALL)){
    //         fiveBall = true;
    //         path = Paths.TERMINAL_3_BALL;
    //     }

    //     if(path.equals(Paths.HANGAR_2_BALL)){
    //         Subsystems.getIntake().setB1(true);
    //     }

    //     Subsystems.getDriveTrain().setOdometry(startPoints[path.index]);
    //     Subsystems.getDriveTrain().setHeading(paths[path.index].getInitialState().holonomicRotation);

    //     Auto.timer.reset();
    //     Auto.timer.start();
    // }
    Double currentTime;

    public PathPlannerTrajectory testPath = PathPlanner.loadPath("overStation", 2.5, 1.5);

    public void init461(){
        //Subsystems.getDriveTrain().pigeon.setYaw(0);
        timer.reset();
        currentTime = 0.0;

        Subsystems.getDriveTrain().setOdometry(testPath.getInitialPose());
        Subsystems.getDriveTrain().setHeading(testPath.getInitialState().holonomicRotation);
        
        //Subsystems.getDriveTrain().pigeon.setYaw(0);
    }

    
    public void run461() {
        
        timer.start();
        double currentTime = Auto.timer.get();
        System.out.println(currentTime);

        Pose2d currentOdomPos = Subsystems.getDriveTrain().getOdometryPoseMeters();
        

        PathPlannerState testState = (PathPlannerState) testPath.sample(currentTime);        
        

        ChassisSpeeds speeds = controller.calculate(currentOdomPos, testState,
                testState.holonomicRotation);
        Subsystems.getDriveTrain().drive(-speeds.vyMetersPerSecond, -speeds.vxMetersPerSecond, -speeds.omegaRadiansPerSecond, false);

        // if(currentTime > (testPath.getTotalTimeSeconds())) {
        //     //Subsystems.getDriveTrain().drive(0, 0, 0, false);
        //     //Subsystems.getDriveTrain().setXConfig();
        // }
        
    }
    
    /**
     * Runs the path
     */
    //  public void run() {
    //     double currentTime = Auto.timer.get();
    //     if(path.setpoints.length > 0){
        //     currentTime += Math.min(path.setpoints[0].getTime(), 0.0);
    //     }
    //     boolean stopped = false;

    //     if(path.equals(Paths.TERMINAL_3_BALL)){
    //         terminal3Ball();
    //     }else if(path.equals(Paths.TERMINAL_2_BALL)){
    //         terminal2Ball();
    //     }else if(path.equals(Paths.HANGAR_2_BALL)){
    //         hangar2Ball();
    //     }else if(path.equals(Paths.OFF_TARMAC)){
    //         offTarmac();
    //     }/*else if(path.equals(Paths.HANGAR_2_BALL_2)){
    //         hangar2Ball2();
    //     }*/

    //     if (currentTime > paths[path.index].getTotalTimeSeconds()) {
    //         Subsystems.getDriveTrain().drive(0, 0, 0, false);

    //         //5 ball path transition
    //         if(fiveBall && path.equals(Paths.TERMINAL_2_BALL) && Subsystems.getIntake().ballPresent()){

    //             System.out.println("Limelight Aiming");
    //             Limelight.setLEDMode(LEDMode.ON);
    //             Subsystems.getShooter().setShooter(Limelight.getFlywheelSpeed(), Limelight.getHoodAngle(), -0.8);
    //             Subsystems.getShooter().revShooter();
    //             Subsystems.getDriveTrain().drive(0.0, 0.0, Subsystems.getDriveTrain().aimAtHub(), false);

    //             if(Limelight.getAimed()){

    //                 System.out.println("Shooting");

    //                 Subsystems.getShooter().shoot();

    //             }

    //             return;
                
    //         }else if(path.name().equals(Paths.TERMINAL_3_BALL.name()) && fiveBall){

    //             Subsystems.getIntake().stopIntake();

    //             if(Subsystems.getIntake().ballPresent()){

    //                 Limelight.setLEDMode(LEDMode.ON);

    //                 Subsystems.getIntake().stopIntake();

    //                 System.out.println("shooting");

    //                 Subsystems.getShooter().shoot();

    //                 return;

    //             }else{

    //                 Limelight.setLEDMode(LEDMode.OFF);

    //                 System.out.println("Done Shooting");

    //                 // Subsystems.getShooter().stopShooter();

    //             }

    //             path = Paths.TERMINAL_2_BALL;
    //             initPath();

    //             return;
    //         }

    //         if(path.equals(Paths.HANGAR_2_BALL)){

    //             Subsystems.getShooter().shoot();

    //             return;

    //         }else if(!Subsystems.getIntake().ballPresent() && path.equals(Paths.HANGAR_2_BALL)){

    //             System.out.println("Done Shooting!");
    //             Subsystems.getShooter().stopShooter();

    //         }

    //         if(path.equals(Paths.OFF_TARMAC)){

    //             Subsystems.getShooter().shoot();

    //             Subsystems.getIntake().feedShooter();

    //         }

    //         stopped = true;

    //     }
    //     for (int i = 0; i < path.setpoints.length; i++){
    //         Setpoint setpoint = path.setpoints[i];
    //         if(setpoint.isInTime(currentTime)){
    //             setpoint.inTime();
    //         }else{
    //             setpoint.outOfTime();
    //         }
    //     }
    //     Pose2d currentOdomPos = Subsystems.getDriveTrain().getOdometryPoseMeters();
    //     PathPlannerState goal = (PathPlannerState) paths[path.index].sample(currentTime);
    //     ChassisSpeeds speeds = controller.calculate(currentOdomPos, goal,
    //             goal.holonomicRotation);
    //     if(!stopped){
    //         Subsystems.getDriveTrain().drive(-speeds.vyMetersPerSecond, -speeds.vxMetersPerSecond, -speeds.omegaRadiansPerSecond, false);
    //     }
    // }

    // /**
    //  * Stops intake, shooter, and feed
    //  */
    // public void stopAuto(){
    //     Subsystems.getIntake().stopIntake();
    //     Subsystems.getShooter().stopShooter();
    // }

    // /**
    //  * Returns time
    //  * @return
    //  */
    public static Timer getTimer(){
        return Auto.timer;
    }

    // /**
    //  * 3 ball auto commands
    //  * Also first part of 5 ball
    //  */
    // public void terminal3Ball(){
    //     double currTime = Auto.timer.get();

    //     if(currTime < 0.25){//Shoot first ball
    //         Subsystems.getShooter().setShooter(ShooterPositions.FENDER_HIGH);
    //         Subsystems.getShooter().revShooter();
    //     }else if(currTime > 0.26 && currTime < 2.0){
    //         Subsystems.getShooter().shoot();
    //     }else if(currTime < 2.1){//Shot first ball
    //         Subsystems.getShooter().stopShooter();
    //     }else if(currTime < 2.55 && currTime > 2.45){
    //         Subsystems.getShooter().setShooter(ShooterPositions.AUTO_TARMAC);
    //         Subsystems.getPneumatics().intakeOut();
    //     }else if(currTime < 5.0 && currTime > 3.0){//Intake ball 2
    //         Subsystems.getIntake().runIntake(0.40, false, false, false);
    //         Subsystems.getShooter().revShooter();
    //     }else if(currTime < 6.75){//Intake ball 3
    //         // Subsystems.getIntake().setIntakeCurrentLimit(Constants.INTAKE_ERROR_CURR_LIM);
    //         Subsystems.getIntake().runIntake(0.8, false, true, false);
    //         Subsystems.getShooter().revShooter();
    //     }else if(currTime < 6.8){//Shoot 2/3
    //         // Subsystems.getIntake().setIntakeCurrentLimit(Constants.INTAKE_NORM_CURR_LIM);
    //         // Subsystems.getShooter().shoot();
    //     }else if(currTime > 6.9){//Continue to shoot 2/3
    //         //Subsystems.getIntake().runIntake(0.25, false, true, false);
    //         System.out.println("----------------\n---------------\n-------------");
    //         Subsystems.getIntake().stopIntake();
    //         // Subsystems.getShooter().shoot();
    //     }
    // }

    // /**
    //  * Second part of 5 ball auto
    //  */
    // public void terminal2Ball(){
    //     double currTime = Auto.timer.get();

    //     if(currTime < 1.8 && currTime > 1.5){//stop shooting
    //         // Subsystems.getShooter().stopShooter();
    //     }else if(currTime < 3.5){//run intake for balls 4/5 as well as rev up the shooter
    //         Subsystems.getShooter().stopShooter();
    //         Subsystems.getIntake().runIntake(0.6, false, false, false);
    //         // Subsystems.getIntake().setIntakeCurrentLimit(Constants.INTAKE_ERROR_CURR_LIM);
    //         Subsystems.getShooter().setShooter(ShooterPositions.AUTO_TARMAC);
    //         Subsystems.getShooter().revShooter();
    //     }else if(currTime < 5.75){//continue running intake for balls 4/5
    //         Subsystems.getIntake().runIntake(0.6, false, true, false);
    //         Subsystems.getShooter().revShooter();
    //     }
    //     // else if(currTime > 5.5){//shoot balls 4/5
    //     //     // Subsystems.getIntake().setIntakeCurrentLimit(Constants.INTAKE_ERROR_CURR_LIM);
    //     //     Subsystems.getShooter().shoot();
    //     // }

    // }

    // /**
    //  * First part of left side 2 ball auto
    //  */
    // public void hangar2Ball(){
    //     double currTime = Auto.timer.get();

    //     if(currTime < 0.1){
    //         Subsystems.getPneumatics().intakeOut();
    //         Subsystems.getShooter().setShooter(ShooterPositions.TARMAC);
    //         Subsystems.getIntake().runIntake(0.6, false, false, true);
    //     }else if(currTime > 4.0){
    //         Subsystems.getIntake().stopIntake();
    //         // Subsystems.getShooter().shoot();
    //     }

    // }

    // /**
    //  * Second part of left side 2 ball auto
    //  */
    // public void hangar2Ball2(){
    //     double currTime = Auto.timer.get();

    //     if(currTime < 0.1){
    //         Subsystems.getIntake().runIntake(0.6, false, false, false);
    //         Subsystems.getShooter().setShooter(ShooterPositions.FENDER_LOW);
    //     }else if(currTime > 2.0){
    //         Subsystems.getIntake().stopIntake();
    //         Subsystems.getShooter().shoot();
    //     }
    // }

    // public void offTarmac(){

    //     double currTime = Auto.timer.get();

    //     if(currTime > 0){

    //         Subsystems.getShooter().setShooter(ShooterPositions.TARMAC);
    //         Subsystems.getShooter().revShooter();

    //     }

    // }
}
