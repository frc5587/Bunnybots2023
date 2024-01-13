package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class BottomAll extends Command {
    private Wrist wrist;
    private Elevator elevator;


    public BottomAll(Wrist wrist, Elevator elevator) {
        this.wrist = wrist;
        this.elevator = elevator;

        addRequirements(wrist, elevator);
    }


    @Override
    public void execute() {
        wrist.setGoal(WristConstants.BOTTOM_SETPOINT);
        elevator.setGoal(ElevConstants.BOTTOM_POSITION);
    }

    @Override
    public boolean isFinished() {
        return wrist.getController().atGoal() && elevator.getController().atGoal();
    }
}
