package frc.robot.Auto.Setpoint_Tests;

public abstract class Setpoint {
    protected Runnable startMethod;
    protected Runnable duringMethod;
    protected Runnable endMethod;
    protected boolean started = false;

    public Setpoint(Runnable startAction, Runnable duringAction, Runnable endAction){
        this.startMethod = startAction;
        this.duringMethod = duringAction;
        this.endMethod = endAction;
    }

    public abstract void inTime();
    public abstract void outTime();
    public abstract boolean isInTime(double currentTime);
}
