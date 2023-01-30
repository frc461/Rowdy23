package frc.robot.Auto.Setpoint_Tests;

import java.util.function.Supplier;

public class WhileTrueSetpoint extends Setpoint {
    private Supplier<Boolean> duringCondition;
    public WhileTrueSetpoint(Supplier<Boolean> duringCondition, Runnable startAction, Runnable duringAction, Runnable endAction){
        super(startAction, duringAction, endAction);
        this.duringCondition = duringCondition;
    }

    public void inTime(){
        if(!started){
            startMethod.run();
            started = false;
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

    /**
     * current time not used here because this setpoint only runs while a supplier returns true
     */
    public boolean isInTime(double currentTime){
        return duringCondition.get();
    }
}
