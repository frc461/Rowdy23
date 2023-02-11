package frc.robot.Subsystems;

public class Subsystems {
    private static DriveTrain driveTrain;
    //private static Climber climber;
    //private static Shooter shooter;
    private static Intake intake;
    private static Elevator elevator;
    //private static Pneumatics pneumatics;

    /**
     * Subsystems class for the purpose of removing the dependancy to the Robot in every constructor 
     */ 
    public static void initSubsystems(){
        driveTrain = new DriveTrain();
        intake = new Intake();
        elevator = new Elevator();
        Subsystems.getElevator().elevatorInit();
    }

    /**
     * gets the drive train
     * @return
     */
    public static DriveTrain getDriveTrain() {
        return driveTrain;
    }
    
    public static Elevator getElevator(){
        return elevator;
    }

    /**
     * Gets the climber
     * @return
     */
    // public static Climber getClimber() {
    //     return climber;
    // }

    /**
     * Gets the intake
     * @return
     */
    public static Intake getIntake() {
        return intake;
    }

    /**
     * Gets the pneumatics
     * @return
     */
    // public static Pneumatics getPneumatics() {
    //     return pneumatics;
    // }

    /**
     * gets the shooter
     * @return
     */
    // public static Shooter getShooter() {
    //     return shooter;
    // }
}
