package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class TeleopElevator extends CommandBase {    
    private Elevator s_Elevator;    
    private DoubleSupplier motionSup;


    public TeleopElevator(Elevator s_Elevator, DoubleSupplier motionSup) {
        this.s_Elevator = s_Elevator;
        addRequirements(s_Elevator);
        this.motionSup = motionSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double motionVal = MathUtil.applyDeadband(motionSup.getAsDouble(), Constants.stickDeadband);
        //check for limit switch always
        s_Elevator.checkLimitSwitches();
        /* Drive */
        if (motionVal != 0) {
            s_Elevator.moveElevator(motionVal);
        } else {
            s_Elevator.holdHeight();
        }
        
    }

    
}
