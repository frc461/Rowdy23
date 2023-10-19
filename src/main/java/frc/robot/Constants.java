package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static double gyroOffset = 0;

    
    public static final double ELEVATOR_P = 0.097;//0.015;
    public static final double ELEVATOR_I = 0.0;
    public static final double ELEVATOR_D = 0.0;//0.001;
    public static final double ELEVATOR_FF = 0.0010000000474974513;
    public static final double ELEVATOR_UPPER_LIMIT = 61;

    //public static final double elevatorStowPos = -100;

    public static final double elevatorHighScore = 61.5;
    public static final double elevatorHighCubeScore = 61.5;
    public static final double elevatorMidScore = 57.7; //79.336;
    public static final double elevatorMidCubeScore = 32.3; //79.336;
    public static final double elevatorConePickup = 0; //6.52;
    public static final double elevatorBot = 0;

    //public static final double wristStowPos = 0.9;

    public static final double WRIST_P = 4.0;
    public static final double WRIST_I = 0.1;
    public static final double WRIST_D = 0.5;
    public static final double WRIST_FF = 0.5;
    public static final double WRIST_LOWER_LIMIT = 0.42;
    public static final double WRIST_MID_LIMIT = 0.63;
    public static final double WRIST_UPPER_LIMIT = 0.86;

    public static final double wristHighConeScore = 0.565; //.613
    public static final double wristHighCubeScore = 0.795;
    public static final double wristMidConeScore = 0.456; //0.607;
    public static final double wristMidCubeScore = 0.755;
    public static final double wristConePickup = 0.468;
    public static final double wristConePickup2 = 0.786; //was 0.8
    public static final double wristCubePickup = 0.590; //was .590 (low) .648 )high)

    public static final class Swerve {
        public static final int pigeonID = 35;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW- (DO NOT USE, ENABLES ROBOT-CENTRIC)

        public static final COTSFalconSwerveConstants chosenModule =  
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(18.75); 
        public static final double wheelBase = Units.inchesToMeters(22.50); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Voltage Compensation */
        public static final double voltageComp = 12.0;

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.16861 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (2.6686 / 12);
        public static final double driveKA = (0.34757 / 12);

        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor =
        wheelCircumference / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.1; //TODO: This must be tuned to specific robot
        public static final double maxAccel = 4.1; //TODO: This must be tuned to specific robot

        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 21;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(292.1); //127.3 original value
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 22;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(22.3);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 13;
            public static final int canCoderID = 23;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(299.9);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 14;
            public static final int canCoderID = 24;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(177.7);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double slowVel = 1.5;
        public static final double slowAccel = 1.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
