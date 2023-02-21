// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;

public class TeleopWrist extends CommandBase {
  private Wrist s_Wrist;
  private DoubleSupplier motionSup;

  /** Creates a new TeleopWrist. */
  public TeleopWrist(Wrist s_Wrist, DoubleSupplier motionSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Wrist = s_Wrist;
    this.motionSup = motionSup;
    addRequirements(s_Wrist);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double motionVal = MathUtil.applyDeadband(motionSup.getAsDouble(), Constants.stickDeadband);
    if (motionVal != 0) {
      s_Wrist.setRotation(s_Wrist.getEncoder().getAbsolutePosition() + motionVal/4);
    } else {
      s_Wrist.setRotation(s_Wrist.getTarget());
    }

  }
}
