package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {

    //Intake motors, ids 7-10(if needed)
        //Beam breaks are true for unbroke and false for broken
    // private DigitalInput shooterBeamBreak = new DigitalInput(16); //top feed
    // private DigitalInput midFeedBeamBreak = new DigitalInput(15); //mid feed
    // private DigitalInput lowFeedBeamBreak = new DigitalInput(14); //close to intake
    // private Lidar hopperLidar1 = new Lidar(20);
    // private Lidar hopperLidar2 = new Lidar(18);
    // private Lidar hopperLidar3 = new Lidar(19);
    
    // //run intake booleans
    // private boolean b1 = false;
    // private boolean b2 = false;
    // private boolean b1InFeed = false;
    // private boolean b1Prepped = false;
    // private boolean b1Mid = false;

    // private double hopperUpperBound = 0.1;
    // private double hopperLowerBound = 0.03;

    // private double currIntakeLim = Constants.INTAKE_NORM_CURR_LIM;
    

    // private CANSparkMax intakeMotor = new CANSparkMax(8, MotorType.kBrushless);
    // private CANSparkMax shooterFeedMotor = new CANSparkMax(9, MotorType.kBrushless);

    private TalonSRX intake1 = new TalonSRX(31);
    private TalonSRX intake2 = new TalonSRX(30);
    private TalonSRX intake3 = new TalonSRX(32);

    private AddressableLED led = new AddressableLED(0);
    private AddressableLEDBuffer ledData = new AddressableLEDBuffer(13);

    public Intake(){
        // intakeMotor.setSmartCurrentLimit(55);
        // shooterFeedMotor.setSmartCurrentLimit(45);
        // shooterFeedMotor.setOpenLoopRampRate(0.5);
        // intakeMotor.burnFlash();
        // shooterFeedMotor.burnFlash(); 
        
             
    }

    //run intake except its actually good
    public void runIntake461(boolean reversed) {
        if (!reversed) {
            intake1.set(ControlMode.PercentOutput, 0.5);
            intake2.set(ControlMode.PercentOutput, -0.5);
            intake3.set(ControlMode.PercentOutput, 0.5);

        } else {
            intake1.set(ControlMode.PercentOutput, -0.5);
            intake2.set(ControlMode.PercentOutput, 0.5);
            intake3.set(ControlMode.PercentOutput, -0.5);
        }
    }

    public void stopIntake461() {
        
        intake1.set(ControlMode.PercentOutput, 0);
        intake2.set(ControlMode.PercentOutput, 0);
        intake3.set(ControlMode.PercentOutput, 0);
    }

    public void showLights(int r, int g, int b) {
        led.setLength(ledData.getLength());
        for (int i = 0; i < ledData.getLength(); i++) {
            ledData.setRGB(i, r, g, b);
        }
        led.setData(ledData);
        led.start();
    }

    public void stopLights() {
        led.setLength(ledData.getLength());
        for (int i = 0; i < ledData.getLength(); i++) {
            ledData.setRGB(i, 0, 0, 0);
        }
        led.setData(ledData);
        led.start();
    }

    /**
     * Run the intake and feed, using beam breaks to figure out when to stop
     * @param triggerVal speed to run intake at
     * @param inverted whether the feed motor is inverted
     * @param override intake override to not use hopperbeam
     * @param intakeOnly to run just the intake and not the feed
     */
    
    
    //  public void runIntake(double triggerVal, boolean inverted, boolean override, boolean intakeOnly){

    //     boolean midBeam = !midFeedBeamBreak.get();
    //     boolean shooterBeam = !shooterBeamBreak.get();
    //     boolean hopperBeam = getHopperBeam();
    //     double feedVal = -0.6;

    //     if(intakeOnly){
    //         intakeMotor.set(triggerVal);
    //         return;
    //     }

    //     if(inverted){
    //         intakeMotor.set(-triggerVal);
    //         shooterFeedMotor.set(-feedVal);
    //         resetBall();
    //         return;
    //     }

    //     if(override){

    //         intakeMotor.set(triggerVal);

    //         if(midBeam || shooterBeam || b1Prepped){
    //             shooterFeedMotor.set(0.0);
    //         }else{
    //             shooterFeedMotor.set(feedVal);
    //         }

    //         if(getIntakeLidar() && b1){
    //             b2 = true;
    //         }

    //         return;

    //     }

    //     //if no ball then just run norms
    //     //if ball in hopper, slow intake speed to allow feed succ
    //     //if ball in feed and not hitting mid beam, run norm speed
    //     //if ball in feed and hit mid beam, slow feed
    //     //run feed til ball hit shooter beam
    //     //only run intake

    //     //////pre 4/5 code//////
        
    //     if(midBeam && !b1Mid){

    //         b1Mid = true;

    //     }else if(b1Mid && midBeam){

    //         feedVal = -0.3;

    //     }else if(b1Mid && !midBeam){

    //         feedVal = 0;
    //         b1InFeed = true;

    //     }

    //     if(b1InFeed && hopperBeam){
    //         b2 = true;
    //     }

    //     if(hopperBeam){
    //         if(!b1){
    //             b1 = true;
    //         }
    //         triggerVal = 0.15;
    //         setIntakeCurrentLimit(55);
    //     }else{
    //         setIntakeCurrentLimit(45);
    //     }

    //     if(getIntakeLidar() && currIntakeLim < 60){
    //         setIntakeCurrentLimit(60);
    //     }

    //     intakeMotor.set(triggerVal);

    //     shooterFeedMotor.set(feedVal);
        
    // }
    
    // public boolean hopperFull(){
    //     return (b1 && b2);
    // }
    // /**
    //  * 
    //  */
    // public void beamBreaksToSmart(){

    //     boolean botBeam = !lowFeedBeamBreak.get();
    //     boolean midBeam = !midFeedBeamBreak.get();
    //     boolean shooterBeam = !shooterBeamBreak.get();


    //     Constants.TUNING_TABLE.putBoolean("botBeam", botBeam);
    //     Constants.TUNING_TABLE.putBoolean("midBeam", midBeam);
    //     Constants.TUNING_TABLE.putBoolean("shooterBeam", shooterBeam);
    //     Constants.TUNING_TABLE.putBoolean("hopBeam", getHopperBeam());

    //     Constants.TUNING_TABLE.putNumber("lidar1", hopperLidar1.getRawDutyCycle());
    //     Constants.TUNING_TABLE.putNumber("lidar2", hopperLidar2.getRawDutyCycle());
    //     Constants.TUNING_TABLE.putNumber("lidar3", hopperLidar3.getRawDutyCycle());

    //     Constants.TUNING_TABLE.putBoolean("intakeLidar", getIntakeLidar());
    //     Constants.TUNING_TABLE.putBoolean("midHopperLidar", getMidHopperLidar());
    //     Constants.TUNING_TABLE.putBoolean("backHopperLidar", getBackHopperLidar());
    //     Constants.TUNING_TABLE.putBoolean("B1", b1);
    //     Constants.TUNING_TABLE.putBoolean("B2", b2);

    //     SmartDashboard.putBoolean("B1", b1);
    //     SmartDashboard.putBoolean("B2", b2);

    // }
    

    // public int getBallCount(){
    //     int count = 0;

    //     if(b1){
    //         count++;
    //     }

    //     if(b2){
    //         count++;
    //     }

    //     return count;
    // }

    // public void shotBall(){

    //     System.out.println("Ball shot\nB1: " + b1 + "\nB2: "  + b2);

    //     if(b1 && b2){
    //         b2 = false;
    //     }else if(b1 && !b2){
    //         b1 = false;
    //     }else if(!b1 && b2){
    //         b2 = false;
    //     }

    // }

    // /**
    //  * Reverses the feed motor
    //  * @param val speed to run the motor at
    //  */
    // public void reverseFeed(double val){
    //     shooterFeedMotor.set(val);
    // }

    // /**
    //  * 
    //  * @return true if a ball is in the robot anywhere there's a sensor, false otherwise
    //  */
    // public boolean ballPresent(){
        
    //     boolean botBeam = !lowFeedBeamBreak.get();
    //     boolean midBeam = !midFeedBeamBreak.get();
    //     boolean shooterBeam = !shooterBeamBreak.get();
    //     boolean hopper = getBallInHopper();
        
    //     return (botBeam || midBeam || shooterBeam || hopper || b1);

    // }

    // public boolean getBallInHopper(){
    //     return 
    //         getIntakeLidar() || getMidHopperLidar() || getBackHopperLidar();
    // }

    // /**
    //  * Resets all the booleans
    //  */
    // public void resetBall(){
    //     b1 = false;
    //     b2 = false;
    //     b1Mid = false;
    //     b1Prepped = false;
    //     b1InFeed = false;
    // }

    // /**
    //  * Completely stops the intake
    //  */
    // public void stopIntake(){
    //     intakeMotor.set(0.0);
    //     stopFeedShooter();
    // }

    // /**
    //  * Runs shooter feed motor 
    //  */
    // public void feedShooter(){
    //     feedShooter(-0.6);
    //     intakeMotor.set(0.15);
    // }

    // /**
    //  * Runs the shooter feed motor 
    //  * @param feedPercent speed to run it at
    //  */
    // public void feedShooter(double feedPercent){
    //     intakeMotor.set(0.15);
    //     shooterFeedMotor.set(feedPercent);
    // }

    // /**
    //  * Stops shooter feed motor
    //  */
    // public void stopFeedShooter(){
    //     shooterFeedMotor.set(0.0);
    // }

    // /**
    //  * Sets the intake motor's current limit
    //  * @param lim the current limit
    //  */
    // public void setIntakeCurrentLimit(int lim){
    //     this.intakeMotor.setSmartCurrentLimit(lim);
    //     currIntakeLim = lim;
    // }

    // public void setIntakeToStuckCurrentLimit(){
    //     setIntakeCurrentLimit(Constants.INTAKE_ERROR_CURR_LIM);
    // }

    // public void setIntakeToUnStuckCurrentLimit(){
    //     setIntakeCurrentLimit(Constants.INTAKE_NORM_CURR_LIM);
    // }

    // public boolean getB1(){
    //     return b1;
    // }

    // public boolean getB2(){
    //     return b2;
    // }

    // public void setB1(boolean val){
    //     b1 = val;
    // }

    // /**
    //  * Gets the current value of the beam break underneath the shooter
    //  * @return whether the beam break is tripped or not
    //  */
    // public boolean getShooterBeam(){
    //     return !(shooterBeamBreak.get());
    // }

    // public boolean getMidBeam(){
    //     return !(midFeedBeamBreak.get());
    // }

    // public boolean getLowBeam(){
    //     return !(lowFeedBeamBreak.get());
    // }

    // public boolean getHopperBeam(){
        
    //     return
    //             //(hopperLidar1.getRawDutyCycle() < hopperUpperBound && hopperLidar1.getRawDutyCycle() > hopperLowerBound)||
    //             (hopperLidar2.getRawDutyCycle() < hopperUpperBound && hopperLidar2.getRawDutyCycle() > hopperLowerBound);
    //             //(hopperLidar3.getRawDutyCycle() < hopperUpperBound && hopperLidar3.getRawDutyCycle() > 0.01);
        
    // }

    // public boolean getIntakeLidar(){
    //     return (hopperLidar1.getRawDutyCycle() > hopperLowerBound) &&
    //             (hopperLidar1.getRawDutyCycle() < hopperUpperBound);
    // }
    
    // public boolean getMidHopperLidar(){
    //     return (hopperLidar2.getRawDutyCycle() > hopperLowerBound) &&
    //             (hopperLidar2.getRawDutyCycle() < hopperUpperBound);
    // }
    
    // public boolean getBackHopperLidar(){
    //     return (hopperLidar3.getRawDutyCycle() > hopperLowerBound) &&
    //             (hopperLidar3.getRawDutyCycle() < hopperUpperBound);
    // }

}
