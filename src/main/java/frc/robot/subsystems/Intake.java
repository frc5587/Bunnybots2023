package frc.robot.subsystems;

import org.frc5587.lib.subsystems.SimpleMotorBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SimpleMotorBase {
    // private static CANSparkMax leftMotor = new CANSparkMax(IntakeConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    // private static CANSparkMax rightMotor = new CANSparkMax(IntakeConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    
    public Intake(CANSparkMax leftMotor, CANSparkMax rightMotor) {
        super(leftMotor, IntakeConstants.FORWARD_THROTTLE, IntakeConstants.REVERSE_THROTTLE);
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }

    public Intake() {
        this(new CANSparkMax(IntakeConstants.LEFT_MOTOR_ID, MotorType.kBrushless), new CANSparkMax(IntakeConstants.RIGHT_MOTOR_ID, MotorType.kBrushless));
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
