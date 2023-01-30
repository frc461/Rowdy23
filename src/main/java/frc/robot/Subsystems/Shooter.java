// package frc.robot.Subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMax.ControlType;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.SparkMaxLimitSwitch.Type;
// import com.revrobotics.SparkMaxPIDController.AccelStrategy;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class Shooter {    
    

//     public enum ShooterPositions{
//         //shootAmt, hoodAmt, feedAmt
//         FENDER_LOW(
//             1500.0, -15.0, -0.75
//         ),
//         FENDER_HIGH(
//             2075.0, -3.5, -0.7 //2250
//         ),
//         TARMAC(
//             2175.0, -16.0, -0.9 //2300, -12.5
//         ),
//         LAUNCHPAD(
//             2615.0, -20, -0.5
//         ),
//         EJECT(
//             1000.0, 0.0, -0.5
//         ),
//         MID_TARMAC(
//             2300.0, -10.0, -0.8
//         ),
//         MID_LAUNCHPAD(
//             2450.0, -16.0, -0.8
//         ),
//         AUTO_TARMAC(
//             2300.0, -15.25, -0.9
//         );

//         public final double shootAmt;
//         public final double hoodAmt;
//         public final double feedAmt;

//         private ShooterPositions(double shootAmt, double hoodAmt, double feedAmt){
//             this.shootAmt = shootAmt;
//             this.hoodAmt = hoodAmt;
//             this.feedAmt = feedAmt;
//         }

//     }

//     private double hoodAmt;
//     private double shooterAmt;
//     private double feedAmt;

//     //Shooter motor. ids 5, 6(follower)
//     private CANSparkMax shooterMotor = new CANSparkMax(5, MotorType.kBrushless);
//     private CANSparkMax shooterFollowerMotor = new CANSparkMax(15, MotorType.kBrushless);
//     private CANSparkMax hoodMotor = new CANSparkMax(10, MotorType.kBrushless);
//     private SparkMaxPIDController shooterPIDController = shooterMotor.getPIDController();
//     private SparkMaxPIDController hoodPIDController = hoodMotor.getPIDController();

//     private boolean ballOffWheel = false;

//     private boolean ballShooting = false;
    
//     private boolean shooterAtSpeed = false;

//     private double shooterP = 0.00004;
//     private double shooterI = 0.0;
//     private double shooterD = 0.0;
//     private double shooterFF = 0.000199;//0.0001963

//     private double shooterVoltageCompensation = 11.0;

//     private double hoodP = 0.0000001;
//     private double hoodI = 0.0;
//     private double hoodD = 0.0;
//     private double hoodFF = 0.0001;

//     public Shooter(){

//         /*
//         hoodMotor.restoreFactoryDefaults(true);
//         shooterMotor.restoreFactoryDefaults(true);
//         shooterFollowerMotor.restoreFactoryDefaults(true);
//         */

//         this.hoodMotor.getEncoder().setPositionConversionFactor(1);

//         hoodPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        
//         hoodPIDController.setP(hoodP);
//         hoodPIDController.setI(hoodI);
//         hoodPIDController.setD(hoodD);
//         hoodPIDController.setFF(hoodFF);

//         hoodPIDController.setSmartMotionMaxAccel(40000.0, 0);
//         hoodPIDController.setOutputRange(-1.25, 1.25);
//         hoodPIDController.setSmartMotionMaxVelocity(40000.0, 0);
//         hoodPIDController.setSmartMotionMinOutputVelocity(0.0, 0);
//         hoodPIDController.setSmartMotionAllowedClosedLoopError(0.0, 0);

//         this.hoodMotor.getEncoder().setPosition(0.0);
//         hoodMotor.setSmartCurrentLimit(20);
//         this.shooterMotor.setInverted(true);
        
//         shooterFollowerMotor.follow(shooterMotor, true);
//         shooterPIDController.setP(shooterP);
//         shooterPIDController.setI(shooterI);
//         shooterPIDController.setD(shooterD);
//         shooterPIDController.setFF(shooterFF);

//         shooterPIDController.setSmartMotionMaxVelocity(3000.0, 0);
//         shooterPIDController.setSmartMotionMinOutputVelocity(0.0, 0);
//         shooterPIDController.setSmartMotionMaxAccel(4000.0, 0);
//         shooterPIDController.setSmartMotionAllowedClosedLoopError(5.0, 0);

//         shooterMotor.enableVoltageCompensation(shooterVoltageCompensation);
//         shooterFollowerMotor.enableVoltageCompensation(shooterVoltageCompensation);

//         shooterMotor.setIdleMode(IdleMode.kCoast);
//         shooterFollowerMotor.setIdleMode(IdleMode.kCoast);

//         hoodMotor.burnFlash();
//         shooterMotor.burnFlash();
//         shooterFollowerMotor.burnFlash();

//     }

//     /**
//      * Resets pid values
//      */
//     public void resetPID(){
        
        
//         // hoodPIDController.setP(SmartDashboard.getNumber("Rotation Motor P", 0.0001));
//         // hoodPIDController.setI(SmartDashboard.getNumber("Rotation Motor I", 0.0));
//         // hoodPIDController.setD(SmartDashboard.getNumber("Rotation Motor D", 0.0));
//         // hoodPIDController.setFF(SmartDashboard.getNumber("Rotation Motor F", 0.001));

//         shooterP = Constants.TUNING_TABLE.getNumber("Shooter Motor P", shooterP);
//         shooterI = Constants.TUNING_TABLE.getNumber("Shooter Motor P", shooterI);
//         shooterD = Constants.TUNING_TABLE.getNumber("Shooter Motor P", shooterD);
//         shooterFF = Constants.TUNING_TABLE.getNumber("Shooter Motor P", shooterFF);

//         shooterPIDController.setP(shooterP);
//         shooterPIDController.setI(shooterI);
//         shooterPIDController.setD(shooterD);
//         shooterPIDController.setFF(shooterFF);

//     }

//     /**
//      * Starts the shooter wheel based on the shooter amount variable that is determined by the dpad
//      */
//     public void shoot(){

//         ballOffWheel = true;

//         if(Subsystems.getIntake().getShooterBeam()){
//             ballShooting = true;
//         }

//         if(ballShooting && !Subsystems.getIntake().getShooterBeam()){
//             Subsystems.getIntake().shotBall();
//             ballShooting = false;
//         }

//         shooterMotor.getPIDController().setReference(shooterAmt, ControlType.kSmartVelocity);

//         if(shooterMotor.getEncoder().getVelocity() >= shooterAmt - (Constants.SHOOTER_DEADZONE) &&
//             shooterMotor.getEncoder().getVelocity() <= shooterAmt + (Constants.SHOOTER_DEADZONE) &&
//             getHoodAtPosition()){
//             shooterAtSpeed = true;

//         }

//         if(shooterMotor.getEncoder().getVelocity() < shooterAmt - (Constants.SHOOTER_FEED_DEADZONE) ||
//         shooterMotor.getEncoder().getVelocity() > shooterAmt + (Constants.SHOOTER_FEED_DEADZONE)){
//             shooterAtSpeed = false;
//         }        

//         if(shooterAtSpeed){

//            // Subsystems.getIntake().feedShooter(feedAmt);
//         }else{
//             //Subsystems.getIntake().stopIntake();
//             //Subsystems.getIntake().stopFeedShooter();
//         }

//     }

//     /**
//      * Stops the shooter
//      */
//     public void stopShooter(){
//         this.shooterMotor.set(0);
//         Subsystems.getIntake().stopFeedShooter();
//         Subsystems.getIntake().stopIntake();
//         Subsystems.getIntake().resetBall();
//         shooterAtSpeed = false;
//         ballOffWheel = false;
//     }

//      /**
//      * Pushes the hood down until it hits the limit switch to 0 it
//      */
//     public boolean fixHood(){

//         if(!hoodMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed()){
//             hoodMotor.set(0.1);
//             return true;
//         }else{
//             hoodMotor.getEncoder().setPosition(0);
//             setShooter(ShooterPositions.EJECT);
//             return false;//return false once done
//         }

//     }

//     /**
//      * Pushes hood and shooter data to Smart Dashboard
//      */
//     public void putShooterDataToDashboard(){
//         Constants.TUNING_TABLE.putNumber("Hood Position", hoodMotor.getEncoder().getPosition());
//         Constants.TUNING_TABLE.putNumber("Hood Error", hoodAmt - hoodMotor.getEncoder().getPosition());
//         Constants.TUNING_TABLE.putNumber("Hood Velocity", hoodMotor.getEncoder().getVelocity());
//         Constants.TUNING_TABLE.putNumber("Shooter Velocity", shooterMotor.getEncoder().getVelocity());
//         Constants.TUNING_TABLE.putNumber("Shooter Error", shooterAmt - shooterMotor.getEncoder().getVelocity());
//         Constants.TUNING_TABLE.putNumber("Hood Setpoint", 0.0);
//         Constants.TUNING_TABLE.putNumber("Shooter Percent", 0.0);
        
//         Constants.TUNING_TABLE.putNumber("Shooter Setpoint", shooterAmt);
//         Constants.TUNING_TABLE.putNumber("Hood Setpoint", hoodAmt);

//     }

//     /**
//      * Sets shooterAmt and hoodAmt variables to specified values in an array who's index is determined by
//      * the inputted dpad value
//      * 
//      * @param pov pov from an XboxController
//      */
//     public void setShooter(int pov){
//         int index = pov / 90;

//         ShooterPositions[] shooterSetpoints = ShooterPositions.values();

//         setShooter(shooterSetpoints[index].shootAmt, shooterSetpoints[index].hoodAmt, shooterSetpoints[index].feedAmt);

//     }

//     /**
//      * Sets all of the parameters of the shooter
//      * @param shooterAmount speed to run the shooter at
//      * @param hoodAmount position to set the hood to
//      * @param feedAmount speed to run the feed motor at
//      */
//     public void setShooter(double shooterAmount, double hoodAmount, double feedAmount){

//         shooterAmt = shooterAmount;
//         hoodAmt = hoodAmount;
//         feedAmt = feedAmount;
//         setHood();

//     }

//     /**
//      * Set the position of the shooter through an enum
//      * @param shooterPosition the enum
//      */
//     public void setShooter(ShooterPositions shooterPosition){

//         setShooter(shooterPosition.shootAmt, shooterPosition.hoodAmt, shooterPosition.feedAmt);

//     }

//     /**
//      * Revs up the shooter, but will not shoot. Used so we can shoot sooner in auto
//      */
//     public void revShooter(){

//         shooterMotor.getPIDController().setReference(shooterAmt, ControlType.kSmartVelocity);

//     }

//     /**
//      * Updates the shooter and hood from smart dashboard
//      */
//     public void updateShooter() {

//         shooterAmt = Constants.TUNING_TABLE.getNumber("Shooter Velocity Set", 0.0);
//         hoodAmt = Constants.TUNING_TABLE.getNumber("Hood Setpoint", 0.0);
//         feedAmt = -0.6;

//         Constants.THETA_A = Constants.TUNING_TABLE.getNumber("Theta A", Constants.THETA_A);
//         Constants.THETA_B = Constants.TUNING_TABLE.getNumber("Theta B", Constants.THETA_B);
//         Constants.THETA_C = Constants.TUNING_TABLE.getNumber("Theta C", Constants.THETA_C);

//         Constants.OMEGA_A = Constants.TUNING_TABLE.getNumber("Omega A", Constants.OMEGA_A);
//         Constants.OMEGA_B = Constants.TUNING_TABLE.getNumber("Omega B", Constants.OMEGA_B);
//         Constants.OMEGA_C = Constants.TUNING_TABLE.getNumber("Omega C", Constants.OMEGA_C);

//     }

//     /**
//      * Sets hood position
//      */
//     public void setHood(){
//         hoodMotor.getPIDController().setReference(hoodAmt, ControlType.kSmartMotion);
//     }


//     /**
//      * Set shooterMotor from smart dashboard
//      */
//     public void tuneShoot(){
//         this.shooterMotor.set(SmartDashboard.getNumber("Shooter Percent", 0.0));
//     }

//     /**
//      * Returns if the hood is within a certain deadzone of its desired position
//      * @return
//      */
//     public boolean getHoodAtPosition(){
//         double hoodPos = hoodMotor.getEncoder().getPosition();

//         return (hoodPos < (hoodAmt - (hoodAmt * Constants.HOOD_DEADZONE)) && hoodPos > (hoodAmt + hoodAmt * (Constants.HOOD_DEADZONE)));

//     }

//     public void reBurnFlash(){

//         hoodMotor.restoreFactoryDefaults(true);
//         shooterMotor.restoreFactoryDefaults(true);
//         shooterFollowerMotor.restoreFactoryDefaults(true);

//         this.hoodMotor.getEncoder().setPositionConversionFactor(1);

//         hoodPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        
//         hoodPIDController.setP(hoodP);
//         hoodPIDController.setI(hoodI);
//         hoodPIDController.setD(hoodD);
//         hoodPIDController.setFF(hoodFF);

//         hoodPIDController.setSmartMotionMaxAccel(40000.0, 0);
//         hoodPIDController.setOutputRange(-1.25, 1.25);
//         hoodPIDController.setSmartMotionMaxVelocity(40000.0, 0);
//         hoodPIDController.setSmartMotionMinOutputVelocity(0.0, 0);
//         hoodPIDController.setSmartMotionAllowedClosedLoopError(0.0, 0);

//         this.hoodMotor.getEncoder().setPosition(0.0);
//         hoodMotor.setSmartCurrentLimit(20);
//         this.shooterMotor.setInverted(true);
        
//         shooterFollowerMotor.follow(shooterMotor, true);
//         shooterPIDController.setP(shooterP);
//         shooterPIDController.setI(shooterI);
//         shooterPIDController.setD(shooterD);
//         shooterPIDController.setFF(shooterFF);

//         shooterPIDController.setSmartMotionMaxVelocity(3000.0, 0);
//         shooterPIDController.setSmartMotionMinOutputVelocity(0.0, 0);
//         shooterPIDController.setSmartMotionMaxAccel(4000.0, 0);
//         shooterPIDController.setSmartMotionAllowedClosedLoopError(5.0, 0);

//         shooterMotor.enableVoltageCompensation(shooterVoltageCompensation);
//         shooterFollowerMotor.enableVoltageCompensation(shooterVoltageCompensation);

//         shooterMotor.setIdleMode(IdleMode.kCoast);
//         shooterFollowerMotor.setIdleMode(IdleMode.kCoast);

//         hoodMotor.burnFlash();
//         shooterMotor.burnFlash();
//         shooterFollowerMotor.burnFlash();

//     }

//     public void resetFactoryDefaults(){

//         shooterMotor.restoreFactoryDefaults(true);
//         shooterFollowerMotor.restoreFactoryDefaults(true);

//     }
// }
