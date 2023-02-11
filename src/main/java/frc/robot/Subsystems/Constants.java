package frc.robot.Subsystems;

import edu.wpi.first.math.util.Units;
import frc.robot.Utilities.ShuffleboardTable;

public class Constants {
    public static final double TALON_FRONT_RIGHT_P = 0.0039;
    public static final double TALON_FRONT_RIGHT_I = 0.0;
    public static final double TALON_FRONT_RIGHT_D = 0.0;
    public static final double TALON_FRONT_RIGHT_FF = 0.001;

    public static final double TALON_FRONT_LEFT_P = 0.0038;
    public static final double TALON_FRONT_LEFT_I = 0.0;
    public static final double TALON_FRONT_LEFT_D = 0.0;
    public static final double TALON_FRONT_LEFT_FF = 0.001;

    public static final double TALON_BACK_RIGHT_P = 0.0038;
    public static final double TALON_BACK_RIGHT_I = 0.0;
    public static final double TALON_BACK_RIGHT_D = 0.0;
    public static final double TALON_BACK_RIGHT_FF = 0.001;

    public static final double TALON_BACK_LEFT_P = 0.0038;
    public static final double TALON_BACK_LEFT_I = 0.0;
    public static final double TALON_BACK_LEFT_D = 0.0;
    public static final double TALON_BACK_LEFT_FF = 0.001;





    public static final double STEER_FRONT_RIGHT_P = 0.01;//0.01; //increases correction factor
    public static final double STEER_FRONT_RIGHT_I = 0.0001;       //attempts to drive error to zero
    public static final double STEER_FRONT_RIGHT_D = 0.0;        //minimize overshoot by slowing slowing correction factor
    public static final double STEER_FRONT_RIGHT_FF = 0.0;         //???

    public static final double STEER_FRONT_LEFT_P = 0.01;//0.01;
    public static final double STEER_FRONT_LEFT_I = 0.0001;
    public static final double STEER_FRONT_LEFT_D = 0.00;
    public static final double STEER_FRONT_LEFT_FF = 0.0;

    public static final double STEER_BACK_RIGHT_P = 0.01;//0.01;
    public static final double STEER_BACK_RIGHT_I = 0.0001;
    public static final double STEER_BACK_RIGHT_D = 0.000;
    public static final double STEER_BACK_RIGHT_FF = 0.00;

    public static final double STEER_BACK_LEFT_P = 0.01;//0.01;
    public static final double STEER_BACK_LEFT_I = 0.0001;
    public static final double STEER_BACK_LEFT_D = 0.00;
    public static final double STEER_BACK_LEFT_FF = 0.0;





    public static final double DRIVE_FRONT_RIGHT_P = 0.0039;
    public static final double DRIVE_FRONT_RIGHT_I = 0.0;
    public static final double DRIVE_FRONT_RIGHT_D = 0.000;
    public static final double DRIVE_FRONT_RIGHT_FF = 0.0;

    public static final double DRIVE_FRONT_LEFT_P = 0.0038;
    public static final double DRIVE_FRONT_LEFT_I = 0.0;
    public static final double DRIVE_FRONT_LEFT_D = 0.000;
    public static final double DRIVE_FRONT_LEFT_FF = 0.001;

    public static final double DRIVE_BACK_RIGHT_P = 0.0038;
    public static final double DRIVE_BACK_RIGHT_I = 0.0;
    public static final double DRIVE_BACK_RIGHT_D = 0.000;
    public static final double DRIVE_BACK_RIGHT_FF = 0.001;

    public static final double DRIVE_BACK_LEFT_P = 0.0038;
    public static final double DRIVE_BACK_LEFT_I = 0.0;
    public static final double DRIVE_BACK_LEFT_D = 0.000;
    public static final double DRIVE_BACK_LEFT_FF = 0.001;

    public static final double ELEVATOR_P = 0.005800000064074993;
    public static final double ELEVATOR_I = 0.0;
    public static final double ELEVATOR_D = 0.0001;
    public static final double ELEVATOR_FF = 0.0010000000474974513;


    public static final double WRIST_P = 0.01;
    public static final double WRIST_I = 0.0001;
    public static final double WRIST_D = 0.00;
    public static final double WRIST_FF = 0.0;

    public static final int FRONT_LEFT_INDEX = 0;
    public static final int FRONT_RIGHT_INDEX = 1;
    public static final int BACK_LEFT_INDEX = 2;
    public static final int BACK_RIGHT_INDEX = 3;

    //rotation offsets 
    public static final int FRONT_LEFT_FORWARD = 0;
    public static final int FRONT_RIGHT_FORWARD = 0;
    public static final int BACK_LEFT_FORWARD = 0;
    public static final int BACK_RIGHT_FORWARD = 0;

    
    public static final double TALON_WHEEL_RADIUS = 1.5;// inches
    public static final double TALON_GEAR_RATIO = 5.25;
    public static final double SECONDS_PER_MINUTE = 60;

    public static final double NEO_WHEEL_RADIUS = 2.0;
    public static final double NEO_DRIVE_GEAR_RATIO = 6.75;
    public static final double NEO_STEER_GEAR_RATIO = 150.0 / 7.0;

    public static final double ENCODER_RESOLUTION = 42;
    public static final double CANCODER_RESOLUTION = 4096;


    public static final double RAD_PER_ROT = 2 * Math.PI;

    public static final boolean TALON_BOT = false;
    public static double MAX_SPEED = 8.8;   //Meters per second :0
    public static final double MAX_ACC = 9;
    public static double MAX_ANGULAR_SPEED = 8 * Math.PI;   //Half rotation per second
    public static final double MAX_ANGULAR_ACC = 2.5 * Math.PI;

    public static final double SHORT_WHEEL_DIST = Units.inchesToMeters(22.75);
    public static final double LONG_WHEEL_DIST = Units.inchesToMeters(19.375);
    public static final double WHEEL_DIST = Constants.TALON_BOT ?  Units.feetToMeters(0.5) : Units.feetToMeters(1);


    public static final double TRIGGER_DEADZONE = 0.1;
    public static final double JSTICK_DEADZONE = 0.11;

    public static final double SHOOTER_DEADZONE = 50.0;
    public static final double SHOOTER_FEED_DEADZONE = 100.0;

    public static final int INTAKE_ERROR_CURR_LIM = 80;
    public static final int INTAKE_NORM_CURR_LIM = 55;

    public static final double RAMP_UP_DEADZONE = 1.0;
    public static final double FIRE_DEADZONE = 5.0;

    public static final double HOOD_MAX = 0.0;
    public static final double HOOD_MIN = -20.0;
    public static final double HOOD_DEADZONE = 0.025;
    public static final double SHOOTER_MAX = 3000.0;
    public static final double SHOOTER_MIN = 0.0;

    public static final float CLIMBER_SOFT_LIMIT = -305.f;

    public static final double LIMELIGHT_DEADZONE = 4.0;
    public static final double LIMELIGHT_ANGLE = 45;
    public static final double LIMELIGHT_HEIGHT = 2.75;
    public static final double GOAL_HEIGHT = 8.5;

    //Limelight calibration

    //22.75
    //19 + 3/8

    //place robot, and then measure from limelight for these values
    private static final double PHI_A = 4.48;
    private static final double PHI_B = -7.85;
    private static final double PHI_C = -15.41;

    //tuned hood angles

    public static double THETA_A = -12.0;
    public static double THETA_B = -15.8;
    public static double THETA_C = -17.25;

    //tuned flywheel speeds
    public static double OMEGA_A = 1925.0;
    public static double OMEGA_B = 2200.0;
    public static double OMEGA_C = 2400.0;


    //y = ax^2 + bx + c
    //a, b, and c values

    public static final double HOOD_ANGLE_A =   THETA_A / ((PHI_A - PHI_B) * (PHI_A - PHI_C)) - THETA_B / ((PHI_A - PHI_B) * (PHI_B - PHI_C)) + THETA_C / ((PHI_A - PHI_C) * (PHI_B - PHI_C));
    public static final double HOOD_ANGLE_B = - THETA_A * (PHI_B + PHI_C) / ((PHI_A - PHI_B) * (PHI_A - PHI_C)) + THETA_B * (PHI_A + PHI_C) / ((PHI_A - PHI_B) * (PHI_B - PHI_C)) + THETA_C * (PHI_A + PHI_B)/ ((PHI_A - PHI_C) * (PHI_C - PHI_B));
    public static final double HOOD_ANGLE_C =   THETA_A * PHI_B * PHI_C / ((PHI_A - PHI_B) * (PHI_A - PHI_C)) - THETA_B * PHI_A * PHI_C / ((PHI_A - PHI_B) * (PHI_B - PHI_C)) + THETA_C * PHI_A * PHI_B / ((PHI_A - PHI_C) * (PHI_B - PHI_C));

    public static final double FLYWHEEL_SPEED_A =   OMEGA_A / ((PHI_A - PHI_B) * (PHI_A - PHI_C)) - OMEGA_B / ((PHI_A - PHI_B) * (PHI_B - PHI_C)) + OMEGA_C / ((PHI_A - PHI_C) * (PHI_B - PHI_C));
    public static final double FLYWHEEL_SPEED_B = - OMEGA_A * (PHI_B + PHI_C) / ((PHI_A - PHI_B) * (PHI_A - PHI_C)) + OMEGA_B * (PHI_A + PHI_C) / ((PHI_A - PHI_B) * (PHI_B - PHI_C)) + OMEGA_C * (PHI_A + PHI_B) / ((PHI_A - PHI_C) * (PHI_C - PHI_B));
    public static final double FLYWHEEL_SPEED_C =   OMEGA_A * PHI_B * PHI_C / ((PHI_A - PHI_B) * (PHI_A - PHI_C)) - OMEGA_B * PHI_A * PHI_C / ((PHI_A - PHI_B) * (PHI_B - PHI_C)) + OMEGA_C * PHI_A * PHI_B / ((PHI_A - PHI_C) * (PHI_B - PHI_C));

    public static ShuffleboardTable TUNING_TABLE;
}
