// package frc.robot.Subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.SoftLimitDirection;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// public class Climber {

//     //Climber motors, ids 15-18. I can't imagine it taking more than 4 motors, nor can I imagine our robot having 18 motors on it
//     private CANSparkMax climberLeft = new CANSparkMax(17, MotorType.kBrushless);
//     private CANSparkMax climberRight = new CANSparkMax(7, MotorType.kBrushless);

//     private int currLimit = 40;

//     public Climber(){

//         climberRight.setSmartCurrentLimit(currLimit);
//         climberLeft.setSmartCurrentLimit(currLimit);
        
//         zeroClimbers();

//         climberRight.burnFlash();
//         climberLeft.burnFlash();

//     }

//     /**
//      * Runs climber motors with two joystick values
//      * @param leftStickAxis - speed to run the left climber at
//      * @param rightStickAxis - speed to run the right climber at
//      */
//     public void runClimbers(double leftStickAxis, double rightStickAxis){
//         climberLeft.set(leftStickAxis);
//         climberRight.set(rightStickAxis);
//     }

//     /**
//      * Zeros the climbers and sets the soft limits
//      */
//     public void zeroClimbers(){
//         climberLeft.getEncoder().setPosition(0.0);
//         climberRight.getEncoder().setPosition(0.0);
//         climberRight.enableSoftLimit(SoftLimitDirection.kForward, false);
//         climberLeft.enableSoftLimit(SoftLimitDirection.kForward, false);
//         climberRight.enableSoftLimit(SoftLimitDirection.kReverse, true);
//         climberLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);
//         climberRight.setSoftLimit(SoftLimitDirection.kReverse, Constants.CLIMBER_SOFT_LIMIT);
//         climberLeft.setSoftLimit(SoftLimitDirection.kReverse, Constants.CLIMBER_SOFT_LIMIT);
//     }

//     /**
//      * Disables soft limits on the climber motors
//      */
//     public void disableSoftLimits(){
//         climberRight.enableSoftLimit(SoftLimitDirection.kReverse, false);
//         climberLeft.enableSoftLimit(SoftLimitDirection.kReverse, false);
//     }
// }
