// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class AutoIntake extends CommandBase {
    /**
     * Creates a new AutoIntakeCmd.
     */

    private final Intake s_Intake;
    private final boolean stop;
    private final boolean cone;
    private final boolean in;

    public AutoIntake(Intake s_Intake, boolean cone, boolean in) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.s_Intake = s_Intake;
        this.cone = cone;
        this.in = in;
        this.stop = false;
        addRequirements(s_Intake);
    }

    public AutoIntake(Intake s_Intake, boolean stop) {
        this.s_Intake = s_Intake;
        this.cone = false;
        this.in = false;
        this.stop = true;
        addRequirements(s_Intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Auto Intake Started");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (stop) {
            s_Intake.setSpeed(0);
        } else {
            if (!cone && in || cone && !in && !(s_Intake.coneBeamBroken() || s_Intake.cubeBeamBroken())) {
                s_Intake.setSpeed(0.7);
            } else if (!(s_Intake.coneBeamBroken() || s_Intake.cubeBeamBroken())) {
                s_Intake.setSpeed(-0.7);
            } else if (!cone) {
                s_Intake.pulseIntake(0.1);
            } else {
                s_Intake.pulseIntake(-0.1);
            }
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
        if (cone) {
            if (in) { return s_Intake.coneBeamBroken(); }
            return !s_Intake.coneBeamBroken();
        } else {
            if (in) { return s_Intake.cubeBeamBroken(); }
            return !s_Intake.cubeBeamBroken();
        }
    }
}
