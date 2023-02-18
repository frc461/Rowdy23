// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class TeleopWrist extends CommandBase {
  private Intake s_Intake;
  private DoubleSupplier motionSup;

  /** Creates a new TeleopWrist. */
  public TeleopWrist(Intake s_Intake, DoubleSupplier motionSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Intake = s_Intake;
    this.motionSup = motionSup;
    addRequirements(s_Intake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double motionVal = MathUtil.applyDeadband(motionSup.getAsDouble(), Constants.stickDeadband);
    if (motionVal != 0) {
      s_Intake.setRotation(s_Intake.getEncoder().getAbsolutePosition() + motionVal/4);
    } else {
      s_Intake.setRotation(s_Intake.getTarget());
    }
  }
}
