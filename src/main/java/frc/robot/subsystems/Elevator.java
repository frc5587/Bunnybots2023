package frc.robot.subsystems;

import org.frc5587.lib.subsystems.ElevatorBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants.ElevConstants;

public class Elevator extends ElevatorBase {
    private static final MotorType kBrushless = null;
    private static ElevatorConstants constants = ElevConstants.constants;
    private static CANSparkMax leftMotor = new CANSparkMax(ElevConstants.LEFT_MOTOR, kBrushless);
    private static CANSparkMax rightMotor = new CANSparkMax(ElevConstants.RIGHT_MOTOR, kBrushless);
    private static MotorControllerGroup motors = new MotorControllerGroup(leftMotor, rightMotor);

    public Elevator() {
        super(constants, motors);
    }

    @Override
    public double getEncoderPosition() {
        return leftMotor.getEncoder().getPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return leftMotor.getEncoder().getVelocity();
    }

    @Override
    public void setEncoderPosition(double position) {
        leftMotor.getEncoder().setPosition(position);
    }

    @Override
    public void configureMotors() {
        leftMotor.restoreFactoryDefaults();
        leftMotor.setInverted(ElevConstants.MOTOR_INVERTED);
        leftMotor.setSmartCurrentLimit(ElevConstants.SUPPLY_LIMIT, ElevConstants.STATOR_LIMIT);
        rightMotor.setSmartCurrentLimit(ElevConstants.SUPPLY_LIMIT, ElevConstants.STATOR_LIMIT);
        
        
    }
}
