// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class AutoIntakeCommand extends CommandBase {
  /** Creates a new AutoIntakeCmd. */

  private Intake s_Intake;
  private final double speed;
  private final boolean cone;
  private final Timer timer;

  public AutoIntakeCommand(Intake s_Intake, double speed, boolean cone) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Intake = s_Intake;
    this.speed = speed;
    this.cone = cone;
    this.timer = new Timer();
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Auto Intake Started");
    timer.start(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Intake.setSpeed(speed);
    System.out.println(timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intake.setSpeed(0);
    System.out.println("Auto Intake Ended");
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (cone) {
      if (speed < 0) {
        return s_Intake.coneBeamBroken();
      }
      return !s_Intake.coneBeamBroken();
    }
    if (speed > 0) {
      return (s_Intake.cubeBeamBroken() && s_Intake.coneBeamBroken());
    }
    return !(s_Intake.cubeBeamBroken() || s_Intake.coneBeamBroken());
  }
}
