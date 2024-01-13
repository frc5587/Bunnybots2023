package frc.robot.subsystems;

import org.frc5587.lib.subsystems.SimpleMotorBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SimpleMotorBase {
    private static CANSparkMax leftMotor = new CANSparkMax(IntakeConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    private static CANSparkMax rightMotor = new CANSparkMax(IntakeConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    
    public Intake() {
        super(leftMotor, IntakeConstants.FORWARD_THROTTLE, IntakeConstants.REVERSE_THROTTLE);
    }

    @Override
    public void configureMotors() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        leftMotor.setInverted(IntakeConstants.LEFT_MOTOR_INVERTED);
        rightMotor.setInverted(IntakeConstants.RIGHT_MOTOR_INVERTED);
        leftMotor.setSmartCurrentLimit(IntakeConstants.STALL_LIMIT, IntakeConstants.FREE_LIMIT);
        rightMotor.setSmartCurrentLimit(IntakeConstants.STALL_LIMIT, IntakeConstants.FREE_LIMIT);
        rightMotor.follow(leftMotor);
    }
}
