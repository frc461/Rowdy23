// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class AutoIntakeCmd extends CommandBase {
  /** Creates a new AutoIntakeCmd. */

  private Intake s_Intake;
  private final double speed;
  private int intakeSetting;
  private boolean intakeOn;

  public AutoIntakeCmd(Intake s_Intake, double speed, boolean intakeOn) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Intake = s_Intake;
    this.speed = speed;
    this.intakeOn = intakeOn;
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {System.out.println("Auto Intake Started");}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if(intakeOn == true){
    s_Intake.setSpeed(speed);
   }else{
    s_Intake.setSpeed(0);
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intake.setSpeed(0);
    System.out.println("Auto Intake Ended");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean i_status = false;
    if (s_Intake.cubeBeamBroken() == false) {
      i_status = true;
    }
    else {
      i_status = false;
  }
    return i_status;
  }
}
