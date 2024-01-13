package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Wrist;

public class TriggerWrist extends Command {
    private Wrist wrist;
    private DoubleSupplier downSupplier, upSupplier;

    public TriggerWrist(Wrist wrist, DoubleSupplier downSupplier, DoubleSupplier upSupplier) {
        this.wrist = wrist;
        this.downSupplier = downSupplier;
        this.upSupplier = upSupplier;
        this.addRequirements(wrist);
    }

    @Override
    public void execute() {
        if(downSupplier.getAsDouble() > 0 && upSupplier.getAsDouble() == 0) {
            wrist.setGoal(mapDown(-downSupplier.getAsDouble()));
        }
        else if(upSupplier.getAsDouble() > 0 && downSupplier.getAsDouble() == 0) {
            wrist.setGoal(mapUp(upSupplier.getAsDouble()));
        }
        else {
            wrist.setGoal(WristConstants.RESTING_SETPOINT);
        }
        SmartDashboard.putNumber("Wrist goal", Math.toDegrees(wrist.getController().getGoal().position));
    }

    private double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    private double mapDown(double x) {
        return map(x, -1, 0, WristConstants.BOTTOM_SETPOINT, WristConstants.RESTING_SETPOINT);
    }

    private double mapUp(double x) {
        return map(x, 0, 1, WristConstants.RESTING_SETPOINT, WristConstants.TOP_SETPOINT);
    }
}
