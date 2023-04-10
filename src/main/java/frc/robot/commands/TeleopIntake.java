// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class TeleopIntake extends CommandBase {
  private final Intake s_Intake;
  private final Joystick operator;
 // private DoubleSupplier vec;
  /** Creates a new TeleopRoller. */
  public TeleopIntake(Intake s_Intake, Joystick operator) {
    this.s_Intake = s_Intake;
    this.operator = operator;
    addRequirements(s_Intake); 
  // Called when the command is initially scheduled.
}

// Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Intake.runIntake(operator);
  }

}
