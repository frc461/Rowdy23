package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public enum LEDMode{
        PIPELINE_CURRENT(0),
        OFF(1),
        BLINK(2),
        ON(3);

        final int tableVal;
        private LEDMode(int tableVal){
            this.tableVal = tableVal;
        }
    };
    private static double[] botPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
    // public static double getZ(){
    //     return table.getEntry("").getDouble(1.0);
    // }

    public static double getRoll(){
        botPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
        return botPose[3];
    }
    public static double getYaw(){
        botPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
        return botPose[4];
    }
    public static double getPitch(){
        botPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
        return botPose[5];
    }

    public static double getTY(){

        return table.getEntry("ty").getDouble(1.0);
    }

    public static double getTX(){
        return table.getEntry("tx").getDouble(0.0);
    }

    public static double getDistanceFeet(){
        return (Constants.GOAL_HEIGHT - Constants.LIMELIGHT_HEIGHT) / Math.tan((SmartDashboard.getNumber("Limelight Ground Angle", 0.0) + Limelight.getTX()) * Math.PI / 180.0);
    }

    public static void setLEDMode(LEDMode mode){
        table.getEntry("ledMode").setNumber(mode.tableVal);
    }

    public static boolean isValidTarget(){
        return ((int) table.getEntry("tv").getDouble(0.0)) != 0;
    }

    public static boolean getAimed(){
        return (Math.abs(Limelight.getTY()) < Constants.LIMELIGHT_DEADZONE && Limelight.isValidTarget());
    }

    public static double getHoodAngle(){
        double x = getTX();
        return Constants.HOOD_ANGLE_A * Math.pow(x, 2.0) + Constants.HOOD_ANGLE_B * x + Constants.HOOD_ANGLE_C;
    }

    public static double getFlywheelSpeed(){
        double x = getTX();
        return Constants.FLYWHEEL_SPEED_A * Math.pow(x, 2.0) + Constants.FLYWHEEL_SPEED_B * x + Constants.FLYWHEEL_SPEED_C;
    }

}