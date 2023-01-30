// package frc.robot.Subsystems;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DutyCycle;

// public class Lidar {
//     private DutyCycle input;

//     public Lidar(int port){
//         this(new DigitalInput(port));
//     }

//     public Lidar(DigitalInput digitalInput){
//         input = new DutyCycle(digitalInput);
//     }

//     public double getRawDutyCycle(){
//         return input.getOutput();
//     }

//     public double getMillimeters(){
//         return getRawDutyCycle() * 4000.0;
//     }

//     public double getInches(){
//         return Units.metersToInches(getMeters());
//     }

//     public double getFeet(){
//         return getInches() / 12.0;
//     }

//     public double getMeters(){
//         return getMillimeters() / 1000.0;
//     }
// }
