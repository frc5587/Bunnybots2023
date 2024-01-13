// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Auto;
import frc.robot.commands.BottomAll;
import frc.robot.commands.DualStickSwerve;
import frc.robot.commands.TopAll;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // SUBSYSTEMS:
    private final Elevator elevator = new Elevator();
    protected final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Wrist wrist = new Wrist();

    // CONTROLLERS:
    private final CommandXboxController xbox = new CommandXboxController(0);
    private final CommandXboxController xbox2 = new CommandXboxController(1);

    // COMMANDS:
    private final DualStickSwerve driveCommand = new DualStickSwerve(swerve, xbox::getLeftY, xbox::getLeftX,
           () -> {return -xbox.getRightX();}, () -> true);
    private final Auto auto = new Auto(wrist, elevator, swerve, intake);
    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    // private final TriggerWrist triggerwWrist = new TriggerWrist(wrist, xbox::getLeftTriggerAxis, xbox::getRightTriggerAxis);
            
  public RobotContainer() {
    swerve.setDefaultCommand(driveCommand);
    autoChooser.addOption("Score and Cross", auto.scoreAndCross);
    autoChooser.addOption("Just Score", auto.justScore);
    autoChooser.addOption("Just Cross", auto.justCross);
    autoChooser.setDefaultOption("Do Nothing", auto.nothing);
    SmartDashboard.putData(autoChooser);
    // wrist.setDefaultCommand(triggerwWrist);
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }
    /* Controller 2 Bindings
        DPad Up = elevatorTop
        DPad Down = elevatorBottom
        Right Bumper = Forward Intake
        Left Bumper = Backward Intake
        A = wristTop
        B = wristMid
        Y = wristBottom
    */
    private void configureBindings() {
        xbox2.povUp().onTrue(new InstantCommand(elevator::elevatorTop));
        xbox2.povLeft().onTrue(new InstantCommand(elevator::elevatorMid));
        xbox2.povRight().onTrue(new InstantCommand(elevator::elevatorMid));
        xbox2.povDown().onTrue(new InstantCommand(elevator::elevatorBottom));
        xbox2.y().onTrue(new InstantCommand(wrist::wristTop));
        xbox2.b().onTrue(new InstantCommand(wrist::wristMid));
        xbox2.x().onTrue(new InstantCommand(wrist::wristMid));
        xbox2.a().onTrue(new InstantCommand(wrist::wristBottom));
        xbox2.rightBumper().whileTrue(new InstantCommand(intake::forward)).onFalse(new InstantCommand(intake::stop));
        xbox2.leftBumper().whileTrue(new InstantCommand(intake::backward)).onFalse(new InstantCommand(intake::stop));
        xbox2.leftTrigger().onTrue(new TopAll(wrist, elevator));
        xbox2.rightTrigger().onTrue(new BottomAll(wrist, elevator));
        xbox2.leftStick().onTrue(new InstantCommand(elevator::resetEncoders));
        xbox2.rightStick().onTrue(new InstantCommand(wrist::resetEncoders));
        xbox.a().whileTrue(
          new ParallelCommandGroup(
            new RunCommand(swerve::stop),
            new RunCommand(swerve::resetModulesToAbsolute)
          )
        ); // in case first method doesn't work 
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
