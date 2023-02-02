package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.BasePigeon;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.SwerveOdometry;

public class DriveTrain {

   
    //drive motors
    private CANSparkMax frontRightDrive = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax frontLeftDrive = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax backLeftDrive = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax backRightDrive = new CANSparkMax(4, MotorType.kBrushless);
    
    private RelativeEncoder frontRightDriveEnc = frontRightDrive.getEncoder();
    private RelativeEncoder frontLeftDriveEnc = frontLeftDrive.getEncoder();
    private RelativeEncoder backLeftDriveEnc = backLeftDrive.getEncoder();
    private RelativeEncoder backRightDriveEnc = backRightDrive.getEncoder();

    //rotation motors (ids: 1, 2, 3, 4)
    private CANSparkMax frontRightRotation = new CANSparkMax(12, MotorType.kBrushless);
    private CANSparkMax frontLeftRotation = new CANSparkMax(11, MotorType.kBrushless);
    private CANSparkMax backLeftRotation = new CANSparkMax(13, MotorType.kBrushless);
    private CANSparkMax backRightRotation = new CANSparkMax(14, MotorType.kBrushless);

    //Talon rotation motors so we can test with old swerve bot
    private WPI_TalonSRX frontRightTalon = new WPI_TalonSRX(12);
    private WPI_TalonSRX frontLeftTalon = new WPI_TalonSRX(11);
    private WPI_TalonSRX backLeftTalon = new WPI_TalonSRX(13);
    private WPI_TalonSRX backRightTalon = new WPI_TalonSRX(14);

    //rotation motor encoders
    private RelativeEncoder frontRightEncoder = frontRightRotation.getEncoder();
    private RelativeEncoder frontLeftEncoder = frontLeftRotation.getEncoder();
    private RelativeEncoder backLeftEncoder = backLeftRotation.getEncoder();
    private RelativeEncoder backRightEncoder = backRightRotation.getEncoder();


    private CANCoder frontRightCANCoder = new CANCoder(22);
    private CANCoder frontLeftCANCoder = new CANCoder(21);
    private CANCoder backLeftCANCoder = new CANCoder(23);
    private CANCoder backRightCANCoder = new CANCoder(24);
                                                            //0.005
    private PIDController aimController = new PIDController(0.01, 0.0, 0.0);

    public final BasePigeon pigeon;

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    //Translation 2d Objects
    private final Translation2d frontRightLocation = new Translation2d(Constants.WHEEL_DIST, -Constants.WHEEL_DIST);
    private final Translation2d frontLeftLocation = new Translation2d( Constants.WHEEL_DIST,  Constants.WHEEL_DIST);
    private final Translation2d backLeftLocation = new Translation2d( -Constants.WHEEL_DIST,  Constants.WHEEL_DIST);
    private final Translation2d backRightLocation = new Translation2d(-Constants.WHEEL_DIST, -Constants.WHEEL_DIST);
    

    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(backLeftLocation, backRightLocation, frontLeftLocation, frontRightLocation);

    private final SwerveOdometry swerveOdometry;

    

    public DriveTrain(){
        backLeftCANCoder.setPositionToAbsolute();
        frontRightCANCoder.setPositionToAbsolute();
        frontLeftCANCoder.setPositionToAbsolute();
        backRightCANCoder.setPositionToAbsolute();

       
        if(Constants.TALON_BOT){
            pigeon = new PigeonIMU(35);
        }else{
            pigeon = new Pigeon2(35);
        }
        this.pigeon.setYaw(0.0);
        
        if(Constants.TALON_BOT){
            frontRightModule = new SwerveModule(this.frontRightDrive, this.frontRightTalon, this.frontRightDriveEnc, 0.0);
            frontLeftModule = new SwerveModule(this.frontLeftDrive, this.frontLeftTalon, this.frontLeftDriveEnc, 0.0);
            backLeftModule = new SwerveModule(this.backLeftDrive, this.backLeftTalon, this.backLeftDriveEnc, 0.0);
            backRightModule = new SwerveModule(this.backRightDrive, this.backRightTalon, this.backRightDriveEnc, 0.0);
            this.initTalons();
        }else{

            putCANCodersToSmartDashboard();
            frontRightModule = new SwerveModule(this.frontRightDrive, this.frontRightRotation, this.frontRightDriveEnc, this.frontRightEncoder, Constants.FRONT_RIGHT_FORWARD);
            frontLeftModule = new SwerveModule(this.frontLeftDrive, this.frontLeftRotation, this.frontLeftDriveEnc, this.frontLeftEncoder, Constants.FRONT_LEFT_FORWARD);
            backLeftModule = new SwerveModule(this.backLeftDrive, this.backLeftRotation, this.backLeftDriveEnc, this.backLeftEncoder, Constants.BACK_LEFT_FORWARD);
            backRightModule = new SwerveModule(this.backRightDrive, this.backRightRotation, this.backRightDriveEnc, this.backRightEncoder, Constants.BACK_RIGHT_FORWARD);
        }
        initSparks();
        //resetModuleRotation();

        this.frontLeftDrive.burnFlash();
        this.frontLeftRotation.burnFlash();
        this.frontRightDrive.burnFlash();
        this.frontRightRotation.burnFlash();
        this.backLeftDrive.burnFlash();
        this.backLeftRotation.burnFlash();
        this.backRightDrive.burnFlash();
        this.backRightRotation.burnFlash();
        
        swerveOdometry = new SwerveOdometry(swerveKinematics, getPigeonHeading());
    }

    /**
     * Method for driving the roboto
     * 
     * @param xSpeed Speed in x direction
     * @param ySpeed Speed in y direction
     * @param rotation Angular speed
     * @param fieldRelative Whether it's in field relative control mode or not
     */
    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative){ 
        SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, -xSpeed, rotation, Rotation2d.fromDegrees(-this.pigeon.getYaw()))
                : new ChassisSpeeds(ySpeed, -xSpeed, rotation)
        );
        this.updateOdometry();
        setSwerveModuleStates(swerveModuleStates);
    }
    
    public void putCANCodersToSmartDashboard(){
        SmartDashboard.putNumber("bl", backLeftCANCoder.getAbsolutePosition());
        SmartDashboard.putNumber("fr", frontRightCANCoder.getAbsolutePosition());
        SmartDashboard.putNumber("fl", frontLeftCANCoder.getAbsolutePosition());
        SmartDashboard.putNumber("br", backRightCANCoder.getAbsolutePosition());
    }

    /**
     * Returns the swerve module state array
     * 
     * @return array of SwerveModuleState 0 is fl, 1 is fr, 2 is bl, 3 is br
     */
    public SwerveModuleState[] getSwerveModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = frontLeftModule.getState();
        states[1] = frontRightModule.getState();
        states[2] = backLeftModule.getState();
        states[3] = backRightModule.getState();
        return states;
    }

    /**
     * Sets each swerve module to its state determined by the states array
     * 
     * @param states an array of the swerve module states. 0 for fl, 1 for fr, 2 for bl, 3 for br 
     */
    public void setSwerveModuleStates(SwerveModuleState[] states){
        if(Constants.TALON_BOT){
            frontLeftModule.setTalonDesiredState(states[Constants.FRONT_LEFT_INDEX]);
            frontRightModule.setTalonDesiredState(states[Constants.FRONT_RIGHT_INDEX]);
            backLeftModule.setTalonDesiredState(states[Constants.BACK_LEFT_INDEX]);
            backRightModule.setTalonDesiredState(states[Constants.BACK_RIGHT_INDEX]);
        }else{
            frontLeftModule.setDesiredState(states[0]);
            frontRightModule.setDesiredState(states[1]);
            backLeftModule.setDesiredState(states[2]);
            backRightModule.setDesiredState(states[3]);
        }
    }

    /**
     * updates the odometry
     */
    public void updateOdometry(){
        swerveOdometry.update(getPigeonHeading(), getSwerveModuleStates());
    }

    /**
     * Sets the current position and rotation of the odometry
     */
    public void setOdometry(Pose2d currentPoint){
        swerveOdometry.resetPosition(currentPoint, getPigeonHeading());
    }

    /**
     * Resets odometry to be the origin (x: 0, y: 0, rotation: 0)
     */
    public void resetOdometry(){
        this.setOdometry(new Pose2d(0.0, 0.0, getPigeonHeading()));
    }
    public boolean checkLevel(){
        if((Subsystems.getDriveTrain().pigeon.getPitch() > -5 && Subsystems.getDriveTrain().pigeon.getPitch() < 5) && (Subsystems.getDriveTrain().pigeon.getRoll() > -5 && Subsystems.getDriveTrain().pigeon.getRoll() < 5)) {
            System.out.println("balanced");
            return true;
        }
        else{
            System.out.println("NOT balanced");
            return false;
        }
    }

    /**
     * Retrieves the current pose meters of the odometry
     * @return pose meters of the odometry
     */
    public Pose2d getOdometryPoseMeters(){
        return swerveOdometry.getPoseMeters();
    }

    /**
     * Returns the heading from our pigeon
     * @return A Rotation2d object of our pigeon heading
     */
    public Rotation2d getPigeonHeading(){
        return Rotation2d.fromDegrees(this.pigeon.getYaw());
    }

    /**
     * Resets the current yaw angle of the robot such that the way it is currently facing is the new forward direction
     */
    public void resetPigeonHeading(){
        this.pigeon.setYaw(0.0);
    }
    public void offsetPigeon(){
        this.pigeon.setYaw(this.pigeon.getYaw()+90);
    }

    /**
     * 
     * @param newHeading
     */
    public void setHeading(Rotation2d newHeading){
        this.setOdometry(new Pose2d(getOdometryPoseMeters().getX(), getOdometryPoseMeters().getY(), newHeading));
    }

    /**
     * Runs each WPI_TalonSRX through the initTalon function
     */
    public void initTalons(){
        initTalon(frontLeftTalon, 0.0);
        initTalon(frontRightTalon, 0.0);
        initTalon(backLeftTalon, 0.0);
        initTalon(backRightTalon, 0.0);
    }

    /**
     * Initates a WPI_TalonSRX with the correct encoder counts and offsets
     * @param talon the motor being initated
     * @param offset the offset of the encoder
     */
    public void initTalon(WPI_TalonSRX talon, double offset){
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);
        Timer.delay(0.3);
        //if the talons get offset, uncomment the lower line, hand set the modules to 0, 
        //redeploy code, and then recomment the line, and redeploy.
        
        // talon.setSelectedSensorPosition(0, 0, 0);
    }

    /**
     * Initializes all the spark maxes
     */
    public void initSparks(){
        if(!Constants.TALON_BOT){
            //drive motors don't need offsets
            initSpark(frontRightDrive, 0.0);
            initSpark(frontLeftDrive, 0.0, false);
            initSpark(backLeftDrive, 0.0, true);
            initSpark(backRightDrive, 0.0, true);
            setPIDF(frontRightDrive, Constants.DRIVE_FRONT_RIGHT_P, Constants.DRIVE_FRONT_RIGHT_I, Constants.DRIVE_FRONT_RIGHT_D, Constants.DRIVE_FRONT_RIGHT_FF);
            setPIDF(frontLeftDrive, Constants.DRIVE_FRONT_LEFT_P, Constants.DRIVE_FRONT_LEFT_I, Constants.DRIVE_FRONT_LEFT_D, Constants.DRIVE_FRONT_LEFT_FF);
            setPIDF(backRightDrive, Constants.DRIVE_BACK_RIGHT_P, Constants.DRIVE_BACK_RIGHT_I, Constants.DRIVE_BACK_RIGHT_D, Constants.DRIVE_BACK_RIGHT_FF);
            setPIDF(backLeftDrive, Constants.DRIVE_BACK_LEFT_P, Constants.DRIVE_BACK_LEFT_I, Constants.DRIVE_BACK_LEFT_D, Constants.DRIVE_BACK_LEFT_FF);

            //negating CANCoder reading to make the encoder read CCW negative so it matches the NEO
            initSpark(frontRightRotation, -(frontRightCANCoder.getAbsolutePosition() % 360.0 - frontRightModule.getOffset()), false);
            initSpark(frontLeftRotation, -(frontLeftCANCoder.getAbsolutePosition() % 360.0 - frontLeftModule.getOffset()), false);
            initSpark(backLeftRotation, -(backLeftCANCoder.getAbsolutePosition() % 360.0 - backLeftModule.getOffset()));
            initSpark(backRightRotation, -(backRightCANCoder.getAbsolutePosition() % 360.0 - backRightModule.getOffset()));
            setPIDF(frontRightRotation, Constants.STEER_FRONT_RIGHT_P, Constants.STEER_FRONT_RIGHT_I, Constants.STEER_FRONT_RIGHT_D, Constants.STEER_FRONT_RIGHT_FF);
            setPIDF(frontLeftRotation, Constants.STEER_FRONT_LEFT_P, Constants.STEER_FRONT_LEFT_I, Constants.STEER_FRONT_LEFT_D, Constants.STEER_FRONT_LEFT_FF);
            setPIDF(backLeftRotation, Constants.STEER_BACK_LEFT_P, Constants.STEER_BACK_LEFT_I, Constants.STEER_BACK_LEFT_D, Constants.STEER_BACK_LEFT_FF);
            setPIDF(backRightRotation, Constants.STEER_BACK_RIGHT_P, Constants.STEER_BACK_RIGHT_I, Constants.STEER_BACK_RIGHT_D, Constants.STEER_BACK_RIGHT_FF);
        }else{
            initSpark(frontRightDrive, 0.0);
            initSpark(frontLeftDrive, 0.0);
            initSpark(backLeftDrive, 0.0);
            initSpark(backRightDrive, 0.0);
    
            setPIDF(frontRightDrive, Constants.TALON_FRONT_RIGHT_P, Constants.TALON_FRONT_RIGHT_I, Constants.TALON_FRONT_RIGHT_D, Constants.TALON_FRONT_RIGHT_FF);
            setPIDF(frontLeftDrive, Constants.TALON_FRONT_LEFT_P, Constants.TALON_FRONT_LEFT_I, Constants.TALON_FRONT_LEFT_D, Constants.TALON_FRONT_LEFT_FF);
            setPIDF(backRightDrive, Constants.TALON_BACK_RIGHT_P, Constants.TALON_BACK_RIGHT_I, Constants.TALON_BACK_RIGHT_D, Constants.TALON_BACK_RIGHT_FF);
            setPIDF(backLeftDrive, Constants.TALON_BACK_LEFT_P, Constants.TALON_BACK_LEFT_I, Constants.TALON_BACK_LEFT_D, Constants.TALON_BACK_LEFT_FF);    
        }
    }

    /**
     * Sets the parameters for the pid controller
     * @param spark the spark max being affected
     * @param p the p value in a PID controller
     * @param i the i value in a PID controller
     * @param d the d value in a PID controller
     * @param f the f (feed forward) value
     */
    private void setPIDF(CANSparkMax spark, double p, double i, double d, double f){
        SparkMaxPIDController pidCon = spark.getPIDController();

        pidCon.setP(p);
        pidCon.setI(i);
        pidCon.setD(d);
        pidCon.setFF(f);
    }

    /**
     * Initializes a single spark max
     */
    public void initSpark(CANSparkMax spark, double error){
        spark.setIdleMode(IdleMode.kBrake);

        error = error < 0.0 ? error + 360.0 : error;
        spark.getEncoder().setPosition(error);
        //spark.getEncoder().setPosition(0.0);
    }

    /**
     * Initializes a single spark max
     */
    public void initSpark(CANSparkMax spark, double error, boolean inverted){
        spark.setIdleMode(IdleMode.kBrake);
        spark.setInverted(inverted);

        error = error < 0.0 ? error + 360.0 : error;
        spark.getEncoder().setPosition(error);
        //spark.getEncoder().setPosition(0.0);
    }

    /**
     * Resets the rotation of the module. Used in the event the NEO encoders accumulate error
     */
    public void resetModuleRotation(){
        frontLeftModule.setOffset(0);
        frontLeftModule.resetRotation();
        frontRightModule.setOffset(0);
        frontRightModule.resetRotation();
        backLeftModule.setOffset(0);
        backLeftModule.resetRotation();
        backRightModule.setOffset(0);
        backRightModule.resetRotation();
    }

    /**
     * Used when tuning the PID on the limelight aiming
     */
    public void resetAimPID(){
        aimController.setPID(SmartDashboard.getNumber("Aim P", 1.0), SmartDashboard.getNumber("Aim I", 0.0), SmartDashboard.getNumber("Aim D", 0.0));
    }
    /**
     * Returns roational value to aim at hub
     * @return 
     */
    public double aimAtHub(){
        double tx = Limelight.getTY();
        return aimController.calculate(tx, 0.0);
    }
    public void turnDegrees(Double inputAngle){
        double curAng = pigeon.getYaw();
        System.out.println(curAng);
        
        while(pigeon.getYaw() < curAng+inputAngle){
            System.out.println(pigeon.getYaw());
            drive(0, 0, -1, false);
        }
     
        // while(pigeon.getYaw() > curAng+inputAngle){
        //     System.out.println(pigeon.getYaw());
        //     drive(0, 0, 1, false);
        // }
        
    }

    /**
     * Sets wheels to be in an 'X' configuration to stop rotation
     */
    public void setXConfig(){
        SwerveModuleState[] xStates = {
            new SwerveModuleState(0.01, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0.01, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0.01, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0.01, Rotation2d.fromDegrees(135))
        };

        setSwerveModuleStates(xStates);
    }

    public double getFLEncoder(){
        return frontLeftDriveEnc.getPosition();
    }

}