package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorDown extends CommandBase{
    
    private Elevator elevator;


    public ElevatorDown(Elevator elevator) {
        this.elevator = elevator;

        addRequirements(elevator);
    }


    @Override
    public void execute() {
        elevator.set(-0.25);
    }

    
    @Override
    public void end(boolean interrupted){
        elevator.set(0);
    }
}
