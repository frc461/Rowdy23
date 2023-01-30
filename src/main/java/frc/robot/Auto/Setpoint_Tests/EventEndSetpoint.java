package frc.robot.Auto.Setpoint_Tests;

import java.util.function.Supplier;

import frc.robot.Auto.Auto;

public class EventEndSetpoint extends Setpoint{
    private double time;
    private Supplier<Boolean> stopCondition;
    private boolean stopTimer = true;
    private boolean inverted = false;
    public EventEndSetpoint(double time, Supplier<Boolean> stopCondition, Runnable startAction, Runnable duringAction, Runnable endAction){
        super(startAction, duringAction, endAction);
        this.stopCondition = stopCondition;
        this.time = time;
    }

    public EventEndSetpoint(double time, boolean stopTimer, Supplier<Boolean> stopCondition, Runnable startAction, Runnable duringAction, Runnable endAction){
        super(startAction, duringAction, endAction);
        this.stopCondition = stopCondition;
        this.stopTimer = stopTimer;
        this.time = time;
    }

    public void inTime(){
        if(stopTimer){
            Auto.getTimer().stop();
        }

        if(!started){
            startMethod.run();
            started = true;
        }else{
            duringMethod.run();
        }
    }

    public void outTime(){
        if(stopTimer){
            Auto.getTimer().start();
        }

        if(started){
            endMethod.run();
            started = false;
        }
    }

    public boolean isInTime(double currentTime){
        double referenceTime = currentTime - time;
        boolean endEventRun = inverted ? !stopCondition.get() : stopCondition.get();
        return referenceTime > 0 && endEventRun;
    }
}