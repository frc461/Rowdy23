package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private double botpose[];

    public void refreshValues(){
        botpose = table.getEntry("botpose").getDoubleArray(new double[6]);
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

    public double getRoll(){
        refreshValues();
        return botpose[3];
    }

    public double getPitch(){
        refreshValues();
        return botpose[4];
    }

    public double getYaw(){
        refreshValues();
        return botpose[5];
    }

    public Pose3d getPose() {
        return new Pose3d(new Translation3d(getRX(), getRY(), getRZ()), new Rotation3d(getRoll(), getPitch(), getYaw()));
    }
}
