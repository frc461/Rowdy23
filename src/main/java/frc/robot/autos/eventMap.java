package frc.robot.autos;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class eventMap {
    public HashMap<String, Command> eventMap = new HashMap<String, Command>();
    private final Swerve s_Swerve;
    private final Intake s_Intake;
    private final Wrist s_Wrist;
    private final Elevator s_Elevator;

    public eventMap(Swerve _s_Swerve, Intake _s_Intake, Wrist _s_Wrist, Elevator _s_Elevator) {
        this.s_Swerve = _s_Swerve;
        this.s_Elevator = _s_Elevator;
        this.s_Intake = _s_Intake;
        this.s_Wrist = _s_Wrist;

        eventMap.put("intakeOff", new AutoIntakeCommand(s_Intake,0, false)); // usually used after pickup cone/cube
        eventMap.put("balance", new InstantCommand(() -> s_Swerve.autoBalance())); //  was just autoBalance *** balance\ on charge station
        eventMap.put("elevatorUp", new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorHighScore)));
        eventMap.put("autoCorrect", new InstantCommand(() -> s_Swerve.rotateToDegree(180)));
        eventMap.put("autoConeOut", new AutoIntakeCommand(_s_Intake, 0.57, true));
        eventMap.put("cubeOut", new AutoIntakeCommand(_s_Intake, -1, false));
        eventMap.put("wristDown", new InstantCommand(() -> s_Wrist.setRotation(.55)));


        eventMap.put(
            "stow",
            Commands.sequence(
            new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)),
            new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT))
            )
        );

        eventMap.put(
            "shootCone",
            Commands.sequence(
            new InstantCommand(() -> s_Elevator.setHeight(130)),
            new InstantCommand(() -> s_Wrist.setRotation(0.73))
            )
        );

        eventMap.put( // picks up cone but does NOT stow automatically
            "intakeCone", 
            Commands.sequence(
            new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorConePickup)),
            new InstantCommand(() -> s_Wrist.setRotation(Constants.wristConePickup)),
            new AutoIntakeCommand(s_Intake, -1, true),
            new WaitCommand(3)
            )
        );

        eventMap.put( // picks up cube but does NOT stow automatically
            "intakeCube",
            Commands.sequence(
            new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)),
            new InstantCommand(() -> s_Wrist.setRotation(Constants.wristCubePickup)),
            new AutoIntakeCommand(s_Intake, 1, false)
            )
        );

        eventMap.put( // scores cone and stows automatically
            "scoreConeHigh",
            Commands.sequence(
            new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorHighScore)),
            new InstantCommand(() -> s_Wrist.setRotation(Constants.wristHighConeScore)),
            new WaitCommand(0.75),
            new AutoIntakeCommand(s_Intake, 1, true),
            new WaitCommand(0.5),
            new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)),
            new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT))
            )
        );

        eventMap.put( // scores cube and stows automatically
            "scoreCubeHigh", 
            Commands.sequence(
            new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorHighScore)),
            new InstantCommand(() -> s_Wrist.setRotation(Constants.wristHighCubeScore)),
            new WaitCommand(0.75), //TODO could be slower
            //new PrintCommand("setting intake"),
            new AutoIntakeCommand(s_Intake, -1, false),
            new WaitCommand(0.5),
            new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)),
            new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT))
            )
        );

        eventMap.put( // scores cube and stows automatically
            "scoreCubeMid", 
            Commands.sequence(
            new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorMidCubeScore)),
            new InstantCommand(() -> s_Wrist.setRotation(Constants.wristMidCubeScore)),
            new WaitCommand(0.5), //TODO could be slower
            //new PrintCommand("setting intake"),
            new AutoIntakeCommand(s_Intake, -1, false),
            new WaitCommand(0.5),
            new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)),
            new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT))
            )
        );

    }

    public HashMap<String, Command> getMap() {
        return eventMap;
    }

}
