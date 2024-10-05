// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.control.DeadbandCommandXboxController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.Auto;
import frc.robot.commands.BottomAll;
import frc.robot.commands.DualStickSwerve;
import frc.robot.commands.TopAll;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
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
    private final Limelight limelight = new Limelight();
    private final Elevator elevator = new Elevator();
    protected final Swerve swerve = new Swerve(limelight);
    private final Intake intake = new Intake();
    private final Wrist wrist = new Wrist();

    // CONTROLLERS:
    private final DeadbandCommandXboxController xbox = new DeadbandCommandXboxController(0, 0.2);
    private final DeadbandCommandXboxController xbox2 = new DeadbandCommandXboxController(1, 0.2);

    // COMMANDS:
    private final DualStickSwerve driveCommand = new DualStickSwerve(swerve, xbox::getLeftY, xbox::getLeftX,
           () -> {return -xbox.getRightX();}, () -> xbox.rightBumper().negate().getAsBoolean());
    private final Auto auto = new Auto(wrist, elevator, swerve, intake);
    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
            
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
        DPad Up = Elevator Top Position
        DPad Down = Elevator Bottom Position
        DPad Left/Right = Elevator Middle Position
        Right Bumper = Forward Intake
        Left Bumper = Backward Intake
        A = Wrist Top Position
        B = Wrist Middle Position
        Y = Wrist Bottom Position
        Left Trigger = Top All (Wrist, Elevator)
        Right Trigger = Bottom All (Wrist, Elevator)
        Left Stick (pressed down) = Reset Elevator Encoders
        Right Stick (pressed down) = Reset Wrist Encoders
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
