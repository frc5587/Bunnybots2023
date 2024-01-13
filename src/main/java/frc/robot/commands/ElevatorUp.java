package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

// Manual ElevatorUp
public class ElevatorUp extends Command {
    
    private Elevator elevator;


    public ElevatorUp(Elevator elevator) {
        this.elevator = elevator;

        addRequirements(elevator);
    }


    @Override
    public void execute() {
        elevator.set(0.25);
    }

    
    @Override
    public void end(boolean interrupted){
        elevator.set(0);
    }
}
