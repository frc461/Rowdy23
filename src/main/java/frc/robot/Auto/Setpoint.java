package frc.robot.Auto;

import java.util.function.Supplier;

public class Setpoint {
    private double startTime;
    private double duration;
    private boolean started = false;
    private Runnable startMethod;
    private Runnable duringMethod;
    private Runnable endMethod;
    private Supplier<Boolean> stopCondition;
    private boolean scheduleBased;
    
    /**
     * 
     * @param time - what time to start the action
     * @param duration - how long to run it
     * @param start - what to do at the very start. Only run once
     * @param during - what to do during. Run consistently
     * @param end - what to do at the very end. Only run once
     */
    public Setpoint(double time, double duration, Runnable start, Runnable during, Runnable end){
        this.startTime = time;
        this.duration = duration;
        this.startMethod = start;
        this.duringMethod = during;
        this.endMethod = end;
        this.scheduleBased = true;
    }

    /**
     * 
     * @param time - time to start our action
     * @param waitTime - amount of time to wait between startAction and endAction
     * @param startAction - what to run at the very beginning. Only run once
     * @param endAction - what to run at the very end. Only run once
     */
    public Setpoint(double time, double waitTime, Runnable startAction, Runnable endAction){
        this(time, waitTime, startAction, Setpoint::noop, endAction);
    }

    /**
     * 
     * @param time - time to run our action
     * @param deadzone - time deadzone after time to run our action
     * @param action - action to be run. Only run once.
     */
    public Setpoint(double time, double deadzone, Runnable action){
        this(time, deadzone, action, Setpoint::noop, Setpoint::noop);

    }
    
    public Setpoint(double time, Supplier<Boolean> endCondition, Runnable startAction, Runnable duringAction, Runnable endAction){
        this.startTime = time;
        this.stopCondition = endCondition;
        this.startMethod = startAction;
        this.duringMethod = duringAction;
        this.endMethod = endAction;
        this.scheduleBased = false;
    }

    /**
     * gets the the time value to start the action
     * @return the time value
     */
    public double getTime(){
        return startTime;
    }

    /**
     * runs the actions for when we are in time for running the action
     * runs the start method once, and then the during method for the remainder
     */
    public void inTime(){
        if(!started){
            if(!scheduleBased){
                Auto.getTimer().stop();
            }
            started = true;
            startMethod.run();
        }else{
            duringMethod.run();
        }
    }

    /**
     * runs the action for when we are after running initially
     * runs the end method once
     */
    public void outOfTime(){
        if(started){
            if(!scheduleBased){
                Auto.getTimer().start();
            }
            started = false;
            endMethod.run();
        }
    }

    /**
     * Returns whether or not we are in the time for running our setpoint
     * @param currentTime current time for computing whether we are in the correct time
     * @return whether or not we are in the time for our setpoint to run
     */
    public boolean isInTime(double currentTime){
        // System.out.println("isInTime: " + scheduleBased);
        // System.out.println("startTime: " + startTime);
        double referenceTime = currentTime - startTime;
        if(scheduleBased){
            return referenceTime < duration && referenceTime > 0.0;
        }else{
            return referenceTime > 0.0 && stopCondition.get();
        }
    }

    /**
     * A no operation method. Used in the setpoint constructor for when you don't want anything to run
     */
    public static void noop(){

    }
}
