// package frc.robot.Subsystems;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

// public class Pneumatics {

//     //Solenoid id's 4/5 do nothing
//     private DoubleSolenoid climberSolenoid = new DoubleSolenoid(41, PneumaticsModuleType.REVPH, 3, 2);
//     private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(41, PneumaticsModuleType.REVPH, 1, 0);
//     private Solenoid climbSafetySolenoid = new Solenoid(41, PneumaticsModuleType.REVPH, 6);
//     private DoubleSolenoid extraSolenoids = new DoubleSolenoid(41, PneumaticsModuleType.REVPH, 4, 5);



//     public Pneumatics(){
//         extraSolenoids.set(Value.kReverse);
//         climberSolenoid.set(Value.kReverse);

//         intakeSolenoid.set(Value.kForward);
//         climbSafetySolenoid.set(false);
//     }

//     /**
//      * Puts climbers down
//      */
//     public void climberDown(){
//         climberSolenoid.set(Value.kForward);
//     }

//     /**
//      * Puts climber up
//      */
//     public void climberUp(){
//         climberSolenoid.set(Value.kReverse);
//     }

//     /**
//      * Toggles climber pneumatic
//      */
//     public void toggleClimber(){
//         climberSolenoid.toggle();
//     }

//     /**
//      * Pulls intake back in
//      */
//     public void intakeIn(){
//         intakeSolenoid.set(Value.kForward);
//     }

//     /**
//      * Pushes intake out
//      */
//     public void intakeOut(){
//         intakeSolenoid.set(Value.kReverse);
//     }

//     /**
//      * Toggles intake pneumatic
//      */
//     public void toggleIntake(){
//         intakeSolenoid.toggle();
//     }

//     /**
//      * Returns state of the intake pneumatic
//      * @return current state 
//      */
//     public Value getIntake(){
//         return intakeSolenoid.get();
//     }

//     /**
//      * Extends climb safety
//      */
//     public void extendClimbSafety(){
//         climbSafetySolenoid.set(false);
//     }

//     /**
//      * Retracts climb safety
//      */
//     public void retractClimbSafety(){
//         climbSafetySolenoid.set(true);
//     }

//     /**
//      * Toggles climb safety pneumatic
//      */
//     public void toggleClimbSafety(){
//         climbSafetySolenoid.toggle();
//     }
// }
