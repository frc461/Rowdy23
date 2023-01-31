package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Subsystems.Limelight.LEDMode;

//import frc.robot.Subsystems.Limelight.LEDMode;
//import frc.robot.Subsystems.Shooter.ShooterPositions;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Teleop {
    //Xbox controllers
    private XboxController driveController = new XboxController(0);
    
    //Currently Used Buttons
    //Left Trigger
    //Right Trigger
    //Left Bumber
    //Right Bumper
    //A Button
    //B Button
    //X Button
    //Y Button

    private XboxController opController = new XboxController(1);

    public boolean fieldRelative = false; //dont forget 
    // private boolean intakeStopped = false;
    // private boolean shooterStopped = true;
    // private boolean fixingHood = false;
     private boolean finishAuto = false;

    // private boolean intakeOverride = false;
    private boolean translating = true;
    
    private Intake intake = Subsystems.getIntake();
    private DriveTrain drivetrain = Subsystems.getDriveTrain();
    //private Climber climber = Subsystems.getClimber();
    //private Shooter shooter = Subsystems.getShooter();
    //private Pneumatics pneumatics = Subsystems.getPneumatics();

    
    /**
     * Loop that has all of our inputs to run the robot
     */
    public void run(){
       

        ////////////////////////// Drive ///////////////////////////
        double driveX = driveController.getLeftX();
        double driveY = driveController.getLeftY();
        
        if(driveController.getXButtonPressed()){
            drivetrain.setXConfig();
        }
        else if(driveController.getXButtonReleased()){
            
        }

        if(driveController.getYButtonPressed()){
            if(Subsystems.getDriveTrain().checkLevel() == true){
                Subsystems.getIntake().showLights(0, 255, 0);
            }
            else{
                Subsystems.getIntake().showLights(255, 255, 255);
            }
           
        }
              
        
        if(driveController.getAButtonPressed()){
            Subsystems.getDriveTrain().offsetPigeon();
        }
        if(driveController.getBButtonPressed()){
            Subsystems.getDriveTrain().resetPigeonHeading();
        }


        // if(finishAuto){
            
        //     if(!intakeStopped){
        //         Subsystems.getIntake().stopIntake();
        //         intakeStopped = true;
        //     }

        //     System.out.println("Limelight Aiming");
        //     Limelight.setLEDMode(LEDMode.ON);

        //     Subsystems.getShooter().setShooter(Limelight.getFlywheelSpeed(), Limelight.getHoodAngle(), -0.8);
        //     Subsystems.getShooter().revShooter();

        //     Subsystems.getDriveTrain().drive(0.0, 0.0, Subsystems.getDriveTrain().aimAtHub(), false);

        //     // if(Limelight.getAimed()){

        //     //     System.out.println("Shooting");

        //     //     Subsystems.getShooter().shoot();

        //     // }
            
        //     // if(driveController.getLeftStickButton() /*|| !intake.ballPresent()*/){
        //     //     finishAuto = false;
        //     //     shooter.setShooter(ShooterPositions.TARMAC);
        //     //     shooter.stopShooter();
        //     // }
        //     return;
        // }
        
        if(Math.abs(driveX) <= 0.08){
            driveX = 0;
        }
        if(Math.abs(driveY) <= 0.08){
            driveY = 0;
        }

        double hyp = Math.sqrt(Math.pow(driveX, 2) + Math.pow(driveY, 2));
        double angle = Math.atan2(driveY, driveX);
        hyp = deadzoneEquations(Constants.JSTICK_DEADZONE, hyp);

        driveX = Math.cos(angle) * hyp;
        driveY = Math.sin(angle) * hyp;

        double rotX;
        boolean ledOn = false;
        Limelight.setLEDMode(LEDMode.OFF);

        if(driveController.getLeftTriggerAxis() > Constants.TRIGGER_DEADZONE){
            Limelight.setLEDMode(LEDMode.ON);
        }
        else if(driveController.getLeftTriggerAxis() < Constants.TRIGGER_DEADZONE){
            Limelight.setLEDMode(LEDMode.OFF);
        }

        //     // 3 point shooter calibration code.
        //     s//hooter.setShooter(Limelight.getFlywheelSpeed(), Limelight.getHoodAngle(), -0.8);
            
        //     rotX = drivetrain.aimAtHub();
            
        //     if(Limelight.getAimed()){
        //         translating = false;
        //         drivetrain.setXConfig();
        //         shooter.shoot();
        //         SmartDashboard.putBoolean("Limelight aimed", true);
        //     }else{
        //         translating = true;
        //         shooter.revShooter();
        //         SmartDashboard.putBoolean("Limelight aimed", false);

        //     }
        // }else if(!driveController.getLeftBumper()){
        //     Limelight.setLEDMode(LEDMode.OFF);
            rotX = -driveController.getRightX();
            rotX = Teleop.deadzoneEquations(Constants.JSTICK_DEADZONE, rotX);
        //     translating = true;
        // }else{
        //     rotX = -driveController.getRightX();
        //     rotX = Teleop.deadzoneEquations(Constants.JSTICK_DEADZONE, rotX);
        // }

        double speed = 0;
        double speedConst = 0.15;
        if(driveController.getLeftBumperPressed()){
           
            //Limelight.setLEDMode(LEDMode.ON);
            
           while(Limelight.getTX() > 3){
            speed = Math.abs(speedConst*Limelight.getTX());
            Subsystems.getDriveTrain().drive(0, -speed, 0, false);
           }
           while(Limelight.getTX() < -3){
            speed = Math.abs(speedConst*Limelight.getTX());
            Subsystems.getDriveTrain().drive(0, speed, 0, false);
           }

           //System.out.println("pitch: " + Limelight.getPitch() + " roll: " + Limelight.getRoll() + " yaw: " + Limelight.getYaw());
           
        }
        if(driveController.getRightBumperPressed()){
            double offset = Limelight.getYaw();
            System.out.println(offset);
            double currGyro = drivetrain.pigeon.getYaw();

            while(drivetrain.pigeon.getYaw()-0.5 > currGyro+offset){
                drivetrain.drive(0, 0, 0.85, false);
            }
            while(drivetrain.pigeon.getYaw()+0.5 < currGyro+offset){
                drivetrain.drive(0, 0, -0.85, false);
            }

            // try{
                
            //     while (Limelight.getYaw() > 1) {
            //         Subsystems.getDriveTrain().drive(0, 0, -1, false);
            //     }
            //     while (Limelight.getYaw() < -1) {
            //         Subsystems.getDriveTrain().drive(0, 0, 1, false);
            //     }
            // }
            //drivetrain.pigeon.setYaw(Limelight.getYaw());
            

            
        }
        

        if(translating){
            drivetrain.drive(driveX * Constants.MAX_SPEED, driveY * Constants.MAX_SPEED, rotX * Constants.MAX_ANGULAR_SPEED, fieldRelative);
        }

        ////////////////////// Field Relative Toggle /////////////////////////////
        if(driveController.getStartButtonPressed() /*|| driveController.getAButtonPressed()*/){
            String fieldRelativeOnOrNot;
            fieldRelative = !fieldRelative;
            if(fieldRelative){
                fieldRelativeOnOrNot = "On";
                intake.showLights(0, 0, 255);
            }else{
                fieldRelativeOnOrNot = "Off";
                intake.showLights(255, 0, 0);
            }
            System.out.println("Field relative is: " + fieldRelativeOnOrNot);

        }

       
        
        ///////////////////////// Intake ////////////////////

        
        if(opController.getLeftBumper()){
            intake.runIntake461(false);
            intake.showLights(255, 0, 255); //255, 0, 255
        } else if(opController.getLeftBumperReleased()) {
            intake.stopIntake461();
            intake.stopLights();
        }
        
        if(opController.getRightBumper()) {
            intake.runIntake461(true);
            intake.showLights(255, 200, 0); //255, 255, 0
        } else if(opController.getRightBumperReleased()) {
            intake.stopIntake461();
            intake.stopLights();
        }

        // if(driveController.getRightBumper()){
        //     Subsystems.getIntake().runIntake(0.5, false, false, false);
        // }else if(driveController.getRightBumperReleased()){
        //     intake.stopIntake();
        // }

        // double opRTrigger = Teleop.deadzoneEquations(Constants.TRIGGER_DEADZONE, opController.getRightTriggerAxis());
        // double opLTrigger = Teleop.deadzoneEquations(Constants.TRIGGER_DEADZONE, opController.getLeftTriggerAxis());

        // if(opController.getRightStickButton()){
        //     opRTrigger = 0.15;
        //     System.out.println("Intake = 0.2");
        // }

        // if(opController.getAButtonPressed()){
        //     intakeOverride = !intakeOverride;
        //     SmartDashboard.putBoolean("intake override", intakeOverride);
        // }

        // if(opRTrigger > 0){
        //     intakeStopped = false;
        //     intake.runIntake(opRTrigger, false, intakeOverride, false);
        // }else if(opLTrigger > 0){
        //     intakeStopped = false;
        //     intake.runIntake(opLTrigger, true, intakeOverride, false);
        // }else if(!intakeStopped){
        //     intakeStopped = true;
        //     Subsystems.getIntake().stopIntake();
        // }

        ////////////////// Hood/Shooter //////////////////////////
        // int pov = driveController.getPOV();
        // if(pov >= 0){
        //     Subsystems.getShooter().setShooter(pov);
        //     Subsystems.getShooter().setHood();
        // }
        
        // if(driveController.getRightStickButton()){
        //     shooter.setShooter(ShooterPositions.TARMAC);
        //     shooter.revShooter();
        // }

        // if(driveController.getXButtonPressed()){
        //     shooter.resetFactoryDefaults();
        // }else if(driveController.getXButtonReleased()){
        //     shooter.reBurnFlash();
        // }

        // if(driveController.getLeftStickButton()){
        //     intake.feedShooter();
        // }else if(driveController.getLeftStickButtonReleased()){
        //     intake.stopIntake();
        // }
        // ////////////////////// Climber //////////////////
        // double lClimbSpeed = Teleop.deadzoneEquations(Constants.JSTICK_DEADZONE, opController.getLeftY());
        // double rClimbSpeed = Teleop.deadzoneEquations(Constants.JSTICK_DEADZONE, opController.getRightY());

        // Subsystems.getClimber().runClimbers(rClimbSpeed, lClimbSpeed);
        
        // if(opController.getLeftBumperPressed()){
        //     climber.disableSoftLimits();            
        // }else if(opController.getLeftBumperReleased()){
        //     climber.zeroClimbers();
        // }

        //////////////////// Pneumatics //////////////////
        // if(opController.getYButtonPressed()){
        //     Subsystems.getPneumatics().toggleClimber();
        // }

        // if(opController.getBButtonPressed()){
        //     Subsystems.getPneumatics().toggleIntake();
        // }

        // if(opController.getXButtonPressed()){
        //     intake.setIntakeCurrentLimit(80);
        // }else if(opController.getXButtonReleased()){
        //     intake.setIntakeCurrentLimit(45);
        // }

        // if(driveController.getRightTriggerAxis() > Constants.TRIGGER_DEADZONE){
        //     shooterStopped = false;
        //     Subsystems.getShooter().shoot();
        // }else if(driveController.getLeftTriggerAxis() > Constants.TRIGGER_DEADZONE){
        //     shooterStopped = false;
        //     Subsystems.getShooter().revShooter();
        // }else if(!shooterStopped){
        //     shooterStopped = true;    
        //     Subsystems.getShooter().stopShooter();
        // }

        

        // if(driveController.getYButtonPressed()){
        //     Subsystems.getDriveTrain().resetAimPID();
        //     Subsystems.getShooter().resetPID();
        //     Subsystems.getShooter().updateShooter();
        // }

        // if(driveController.getBackButtonPressed()){
        //     // Subsystems.getShooter().updateShooter();
        //     // Subsystems.getShooter().setHood();
        //     if(fixingHood){
        //         fixingHood = false;
        //     }else{
        //         fixingHood = true;
        //     }
        // }

        // if(fixingHood){
        //     fixingHood = shooter.fixHood();
        // }
        
    }

    /**
     * Calculates a new magnitude of input taking the dead zone into account. This stops it from jumping
     * from 0 to the deadzone value.
     * 
     * @param deadZoneRadius deadzone radius
     * @param hyp magnitude of input
     * @return
     */
    private static double deadzoneEquations(double deadZoneRadius, double hyp){
        if(hyp >= deadZoneRadius){
            return (1/(1-deadZoneRadius)) * (hyp - deadZoneRadius);
        }else if(hyp <= -deadZoneRadius){
            return (1/(1-deadZoneRadius)) * (hyp + deadZoneRadius);
        }
        return 0;
    }

    public void autod(){
        finishAuto = true;
    }
}
