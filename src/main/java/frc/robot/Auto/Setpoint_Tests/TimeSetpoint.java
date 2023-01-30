package frc.robot.Auto.Setpoint_Tests;

public class TimeSetpoint extends Setpoint {
    private double duration;
    private double time;
    public TimeSetpoint(double time, double duration, Runnable startAction, Runnable duringAction, Runnable endAction){
        super(startAction, duringAction, endAction);
        this.duration = duration;
        this.time = time;
    }

    public boolean isInTime(double currentTime){
        double referenceTime = currentTime - time;
        return referenceTime > 0 && referenceTime < duration;
    }

    public void inTime(){
        if(!started){
            startMethod.run();
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
}
