package frc.robot.subsystems;

import frc.robot.Constants.WristConstants;

import org.frc5587.lib.subsystems.PivotingArmBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Wrist extends PivotingArmBase {
    public static CANSparkMax motor = new CANSparkMax(WristConstants.MOTOR_ID, MotorType.kBrushless); 

    private final DigitalInput frontLimitSwitch = new DigitalInput(WristConstants.SWITCH_PORTS[0]);
    private final DigitalInput rearLimitSwitch = new DigitalInput(WristConstants.SWITCH_PORTS[1]);

    public static PivotingArmConstants constants = new PivotingArmConstants(
        WristConstants.GEARING,
        WristConstants.SOFT_LIMITS,
        WristConstants.ZERO_OFFSET,
        WristConstants.ENCODER_CPR,
        WristConstants.SWITCH_PORTS,
        WristConstants.SWITCH_INVERTIONS,
        WristConstants.PID,
        WristConstants.FF
    );

    public Wrist() {
        this(motor);
    }
    
    public Wrist(CANSparkMax motor) {
        super(constants, motor);
    }

    public DigitalInput getFrontLimitSwitch() {
        return frontLimitSwitch;
    }

    public DigitalInput getRearLimitSwitch() {
        return rearLimitSwitch;
    }

    @Override
    public double getEncoderPosition() {
        return motor.getEncoder().getPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return motor.getEncoder().getVelocity();
    }

    @Override
    public void setEncoderPosition(double position) {
        motor.getEncoder().setPosition(position);
    }

    @Override
    public void configureMotors() {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(WristConstants.MOTOR_INVERTED);
        motor.setSmartCurrentLimit(WristConstants.STALL_LIMIT, WristConstants.FREE_LIMIT);
        resetEncoders();
    }
    
    public void forward() {
        if(!getFrontLimitSwitch().get()) {
            motor.set(0.3);
        }
    }

    public void back() {
        if(!getRearLimitSwitch().get()) {
            motor.set(-0.3);
        }
    }
    
    public void moveRear() {
        setGoal(WristConstants.REAR_SETPOINT);
    }
    
    public void moveFront() {
        setGoal(WristConstants.FRONT_SETPOINT);
    }

    public void toggleArm() {
        if(getFrontLimitSwitch().get()) {
            moveRear();
        } else if (getFrontLimitSwitch().get()) {
            moveFront();
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putBoolean("Front Limit Switch", getFrontLimitSwitch().get());
        SmartDashboard.putBoolean("Rear Limit Switch", getRearLimitSwitch().get());
    }
}
