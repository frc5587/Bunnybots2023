package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class TopAll extends Command {
    private Wrist wrist;
    private Elevator elevator;


    public TopAll(Wrist wrist, Elevator elevator) {
        this.wrist = wrist;
        this.elevator = elevator;

        addRequirements(wrist, elevator);
    }


    @Override
    public void execute() {
        wrist.setGoal(WristConstants.TOP_SETPOINT);
        elevator.setGoal(ElevConstants.TOP_POSITION);
    }

    @Override
    public boolean isFinished() {
        return wrist.getController().atGoal() && elevator.getController().atGoal();
    }
}
