package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class Auto extends SequentialCommandGroup {
    public Auto(Wrist wrist, Elevator elevator, Swerve swerve, Intake intake) {
        super(
            new TopAll(wrist, elevator),
            new WaitCommand(1),
            new InstantCommand(intake::forward),
            new WaitCommand(1),
            new InstantCommand(intake::stop),
            new BottomAll(wrist, elevator),
            new WaitCommand(1),
            new ParallelDeadlineGroup(
                new WaitCommand(1.2), 
                new InstantCommand(
                    () -> {swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, -(3.14 / 4)));} // move pi/4 radians per second for one second (with time allotted for acceleration), i.e. move 45 degrees total. CCW+ so it needs to be negative.
                )
            ),
            new InstantCommand(swerve::stop),
            new WaitCommand(0.5),
            new ParallelDeadlineGroup(
                new WaitCommand(6),
                new InstantCommand(
                    () -> {swerve.setChassisSpeeds(new ChassisSpeeds(1, 0, 0));} // move 1 meter per second for 5.5 seconds (with time allotted for acceleration), i.e. move 5.5 meters total. We're going backwards hence negative.
                ) 
            ),
            new InstantCommand(swerve::stop)
        );
    }
}
