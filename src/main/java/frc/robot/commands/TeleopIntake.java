// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class TeleopIntake extends CommandBase {
  private Intake s_Intake;
  private DoubleSupplier motionSup;
  private boolean outtakeCone;
  /** Creates a new TeleopRoller. */
  public TeleopIntake(Intake s_Intake, DoubleSupplier motionSup, boolean outtakeCone) {
    this.s_Intake = s_Intake;
    this.motionSup = motionSup;
    this.outtakeCone = outtakeCone;
    addRequirements(s_Intake); 
  }

  // Called when the command is initially scheduled.
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(motionSup.getAsDouble()) > Constants.stickDeadband) {
      if (outtakeCone) {
        s_Intake.runIntake461(false);
      } else {
        s_Intake.runIntake461(true);
      }
    } else {
      s_Intake.stopIntake461();
    }
  }

}
