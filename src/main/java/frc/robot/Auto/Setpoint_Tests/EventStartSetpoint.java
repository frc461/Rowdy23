package frc.robot.Auto.Setpoint_Tests;

import java.util.function.Supplier;

import frc.robot.Auto.Auto;

public class EventStartSetpoint extends Setpoint{
    private Supplier<Boolean> beginCondition;
    private double duration;
    private double startTime;

    public EventStartSetpoint(Supplier<Boolean> beginCondition, double duration, Runnable startAction, Runnable duringAction, Runnable endAction){
        super(startAction, duringAction, endAction);
        this.beginCondition = beginCondition;
        this.duration = duration;
    }

    public void inTime(){
        if(!started){
            startMethod.run();
            startTime = Auto.getTimer().get();
            started = true;
        }else{
            duringMethod.run();
        }
    }

    public void outTime(){
        if(started){
            endMethod.run();
            started = false;
        }
    }

    public boolean isInTime(double currentTime){
        if(!started){
            started = beginCondition.get();
        }
        double referenceTime = currentTime - startTime;
        return started && referenceTime < duration;
    }
}
