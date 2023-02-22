// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class TeleopIntake extends CommandBase {
  private Intake s_Intake;
 // private DoubleSupplier vec;
  /** Creates a new TeleopRoller. */
  public TeleopIntake(Intake s_Intake, DoubleSupplier motionSup) {
    this.s_Intake = s_Intake;
    addRequirements(s_Intake); 
  
  // Called when the command is initially scheduled.


}

// Called every time the scheduler runs while the command is scheduled.


  //TODO dont forget this override thiny

  @Override
  public void execute() {
     
    System.out.println(speed);

  if(speed == 0 && s_Intake.coneBeamBroken() == true){
    s_Intake.pulseIntake(-.1);
    System.out.println("pulse1");
  }
  else if(speed == 0 && s_Intake.cubeBeamBroken() == true){
    s_Intake.pulseIntake(.1);
    System.out.println("pulse2");
  }
  else if(speed < 0){
    s_Intake.runIntake(0, false);
  }
  else if(speed > 0){
    s_Intake.runIntake(.5, false);
  }
  else{
    s_Intake.runIntake(-.5, false);
  }
    
  }

}
