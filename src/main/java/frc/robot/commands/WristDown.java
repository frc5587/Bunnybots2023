package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

// Manual WristDown
public class WristDown extends CommandBase{

    private Wrist wrist;


    public WristDown(Wrist wrist) {
        this.wrist = wrist;

        addRequirements(wrist);
    }


    @Override
    public void execute() {
        wrist.set(-0.25);
    }

    
    @Override
    public void end(boolean interrupted){
        wrist.set(0);
    }
}
