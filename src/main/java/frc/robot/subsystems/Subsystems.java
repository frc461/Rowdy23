package frc.robot.subsystems;

public class Subsystems {
    private static Intake intake;
    private static Elevator elevator;

    /**
     * Subsystems class for the purpose of removing the dependancy to the Robot in every constructor 
     */ 
    public static void initSubsystems(){
        intake = new Intake();
        elevator = new Elevator();
    }

    /**
     * gets the drive train
     * @return
     */
    // public static DriveTrain getDriveTrain() {
    //     return driveTrain;
    // }
    
    public static Elevator getElevator(){
        return elevator;
    }

    /**
     * Gets the intake
     * @return
     */
    public static Intake getIntake() {
        return intake;
    }
}
