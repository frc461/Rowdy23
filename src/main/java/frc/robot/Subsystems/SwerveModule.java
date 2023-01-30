package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveModule {

    private CANSparkMax driveMotor;
    private CANSparkMax turningMotor;
    private WPI_TalonSRX talonMotor;
    private double offset;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder turningEncoder;

    /**
     * Constructs a new swerve module object
     * 
     * @param driveMotor     drive motor
     * @param turningMotor   turning motor
     * @param driveEndoder   drive encoder
     * @param turningEncoder turning encoder
     */
    public SwerveModule(
            CANSparkMax driveMotor,
            CANSparkMax turningMotor,
            RelativeEncoder driveEncoder,
            RelativeEncoder turningEncoder,
            double offset) {
        this.driveMotor = driveMotor;
        this.driveEncoder = driveEncoder;
        this.turningMotor = turningMotor;
        this.turningEncoder = turningEncoder;
        this.offset = offset;

        driveEncoder.setVelocityConversionFactor(Constants.RAD_PER_ROT * Units.inchesToMeters(Constants.NEO_WHEEL_RADIUS)
        / Constants.NEO_DRIVE_GEAR_RATIO / Constants.SECONDS_PER_MINUTE);
        turningEncoder.setPositionConversionFactor(360.0 / Constants.NEO_STEER_GEAR_RATIO);
    }

    /**
     * Swerve module constructor for talons
     * 
     * @param driveMotor   CANSparkMax
     * @param turningMotor WPI_TalonSRX
     * @param driveEncoder RelativeEncoder
     */
    public SwerveModule(
            CANSparkMax driveMotor,
            WPI_TalonSRX turningMotor,
            RelativeEncoder driveEncoder,
            double offset) {
        this.driveMotor = driveMotor;
        this.talonMotor = turningMotor;
        this.driveEncoder = driveEncoder;
        this.offset = offset;
        driveEncoder.setVelocityConversionFactor(Constants.RAD_PER_ROT * Units.inchesToMeters(Constants.TALON_WHEEL_RADIUS)
        / Constants.TALON_GEAR_RATIO / Constants.SECONDS_PER_MINUTE);
    }

    /**
     * Returns the rotational offset of a swerve module
     * @return
     */
    public double getOffset() {
        return offset;
    }

    /**
     * Sets the rotational offset of a swerve module
     * @param offset
     */
    public void setOffset(double offset)
    {
        this.offset = offset;
    }

    /**
     * Reset the rotation of a swerve module to 0
     */
    public void resetRotation(){
        this.turningEncoder.setPosition(0);
    }

    /**
     * Returns swerve module state object
     * @return SwerveModuleState
     */
    public SwerveModuleState getState() {
        if (Constants.TALON_BOT) {
            return new SwerveModuleState(-driveEncoder.getVelocity(), Rotation2d
                    .fromDegrees(getHeading().getDegrees() % 1.0));
        } else {
            return new SwerveModuleState(-driveEncoder.getVelocity(),
                    getHeading());
        }
    }

    /**
     * Sets the desired state of the swerve module
     * @param desiredState
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize to avoid spinning over 90 degrees, or pi/2 radians
        SwerveModuleState state = SwerveModule.optimize(desiredState,
                getHeading());
        driveMotor.getPIDController().setReference(Units.metersToInches(state.speedMetersPerSecond)
        * Constants.SECONDS_PER_MINUTE / (Constants.NEO_WHEEL_RADIUS * Constants.RAD_PER_ROT) / Constants.NEO_DRIVE_GEAR_RATIO, ControlType.kVelocity);   

        if(state.speedMetersPerSecond != 0.0){
            turningMotor.getPIDController().setReference(state.angle.getDegrees(), ControlType.kPosition);
        }
    }

    /**
     * Sets the module to the desired state, this is the one to use for talons
     * @param desired The desired SwerveModuleState
     */
    public void setTalonDesiredState(SwerveModuleState desired) {
        // Optimize to avoid spinning over 90 degrees, or pi/2 radians
        SwerveModuleState state = SwerveModule.optimize(desired,
                getHeading());

        driveMotor.getPIDController().setReference(Units.metersToInches(state.speedMetersPerSecond)
         * Constants.SECONDS_PER_MINUTE / (Constants.TALON_WHEEL_RADIUS * Constants.RAD_PER_ROT) / Constants.TALON_GEAR_RATIO, CANSparkMax.ControlType.kVelocity);

        setHeading(state);
    }

    /**
     * Returns heading of robot
     * @return
     */
    public Rotation2d getHeading() {
        if (Constants.TALON_BOT) {
            double heading = (talonMotor.getSelectedSensorPosition() / Constants.ENCODER_RESOLUTION) * 360.0;
            return Rotation2d.fromDegrees(heading);
        } else {
            return Rotation2d.fromDegrees(turningEncoder.getPosition());
        }
    }

    /**
     * Sets the turning motors to angle specified in the SwerveModuleState
     * @param state
     */
    public void setHeading(SwerveModuleState state) {
        if(state.speedMetersPerSecond == 0.0) return;
        double currentHeadingRadians = getHeading().getRadians();
        double setPointHeadingRadians = state.angle.getRadians();

        // System.out.printf("Module %d: Set Points Equal?: %b\n", this.moduleId, (Math.abs(optimizedSetPoint - setPointHeadingRadians) < 1e-4));
        double deltaRadians = setPointHeadingRadians - currentHeadingRadians;
        double deltaPulses = deltaRadians / ((2.0 * Math.PI) / Constants.ENCODER_RESOLUTION);

        double currentPulses = talonMotor.getSelectedSensorPosition();
        double referencePulses = currentPulses + deltaPulses;

        talonMotor.set(ControlMode.MotionMagic, referencePulses);
    }

    /**
     * Optimizes swerve drive angle to require least amount of turning on the turning motor
     * @param desiredState
     * @param currentAngle
     * @return
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        boolean inverted = false;

        double desiredDegrees = desiredState.angle.getDegrees() % 360.0;
        if(desiredDegrees < 0.0){
            desiredDegrees += 360.0;
        }

        double currentDegrees = currentAngle.getDegrees();
        double currentMod = currentDegrees % 360.0;
        if(currentMod < 0.0){
            currentMod += 360.0;
        }

        if(Math.abs(currentMod - desiredDegrees) > 90.0 && Math.abs(currentMod - desiredDegrees) <= 270.0){
            inverted = true;
            desiredDegrees -= 180.0;
        }

        double deltaAngle = desiredDegrees - currentMod;
        if(deltaAngle < 0.0){
            deltaAngle += 360.0;
        }

        double counterClockWiseAngle = deltaAngle;
        double clockWiseAngle = deltaAngle - 360.0;

        if(Math.abs(counterClockWiseAngle) < Math.abs(clockWiseAngle)){
            desiredDegrees = counterClockWiseAngle;
        }else{
            desiredDegrees = clockWiseAngle;
        }

        double magnitude = desiredState.speedMetersPerSecond;

        if(inverted){
            magnitude *= -1.0;
        }

        desiredDegrees += currentDegrees;

        return new SwerveModuleState(magnitude, Rotation2d.fromDegrees(desiredDegrees));
    }
}