package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private double[] botpose;

    public void refreshValues(){
        botpose = table.getEntry("botpose").getDoubleArray(new double[6]);
    }

    public double getRX(){
        refreshValues();
        return botpose[0];
    }

    public double getRY(){
        refreshValues();
        return botpose[1];
    }

    public double getRZ(){
        refreshValues();
        return botpose[2];
    }

    public double getRot(){
        refreshValues();
        return botpose[3];
    }

    public double getWhoKnowsWHat(){
        refreshValues();
        return botpose[4];
    }
}
