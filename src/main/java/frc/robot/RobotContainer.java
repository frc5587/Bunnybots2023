// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.control.DeadbandCommandXboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DualStickSwerve;
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
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Wrist wrist = new Wrist();

    // CONTROLLERS:
    private final DeadbandCommandXboxController xbox = new DeadbandCommandXboxController(0);
    private final DeadbandCommandXboxController xbox2 = new DeadbandCommandXboxController(1);

    // COMMANDS:
    private final DualStickSwerve driveCommand = new DualStickSwerve(swerve, xbox::getRightY, xbox::getRightX,
            xbox::getLeftX, () -> false);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        swerve.setDefaultCommand(driveCommand);
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        xbox2.povUp().onTrue(new InstantCommand(elevator::elevatorUp));
        xbox2.povDown().onTrue(new InstantCommand(elevator::elevatorDown));
        xbox2.rightBumper().onTrue(new InstantCommand(wrist::wristUp));
        xbox2.leftBumper().onTrue(new InstantCommand(wrist::wristDown));
        xbox2.a().whileTrue(new InstantCommand(intake::forward));
        xbox2.b().whileTrue(new InstantCommand(intake::backward));
        xbox.x().whileTrue(new InstantCommand(elevator::elevatorUpSlow)).onFalse(new InstantCommand(elevator::stop));
        xbox.y().whileTrue(new InstantCommand(elevator::elevatorDownSlow)).onFalse(new InstantCommand(elevator::stop));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
