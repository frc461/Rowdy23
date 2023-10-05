package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("campose");
    private double botpose[];

    public void refreshValues(){
        botpose = table.getEntry("campose").getDoubleArray(new double[6]);
        System.out.println("refresh");
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

    public double getTZ(){
        refreshValues();
        return botpose[2];
    }

    public double getRot(){
        refreshValues();
        return botpose[3];
    }
}
