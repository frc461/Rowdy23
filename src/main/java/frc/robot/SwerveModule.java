package frc.robot;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;


import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;


    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    private CANCoder angleEncoder;

    private SparkMaxPIDController driveController;
    private SparkMaxPIDController angleController;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = mAngleMotor.getEncoder();
        angleController = mAngleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = mDriveMotor.getEncoder();
        driveController = mDriveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }
    

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput);    
        }
        else {
            driveController.setReference(
            desiredState.speedMetersPerSecond,
            ControlType.kVelocity,
            0,
            feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    public void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        mAngleMotor.getEncoder().setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        
        mAngleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(mAngleMotor, Usage.kPositionOnly);
        mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
        angleController.setP(Constants.Swerve.angleKP);
        angleController.setI(Constants.Swerve.angleKI);
        angleController.setD(Constants.Swerve.angleKD);
        angleController.setFF(Constants.Swerve.angleKF);
        mAngleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        mAngleMotor.burnFlash();
        resetToAbsolute();
    }

    private void configDriveMotor(){        

        mDriveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(mDriveMotor, Usage.kAll);
        mDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
        driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
        driveController.setP(Constants.Swerve.angleKP);
        driveController.setI(Constants.Swerve.angleKI);
        driveController.setD(Constants.Swerve.angleKD);
        driveController.setFF(Constants.Swerve.angleKF);
        mDriveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        mDriveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    //everything below here is fine.

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            driveEncoder.getVelocity(), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            driveEncoder.getPosition(), 
            getAngle()
        );
    }
}