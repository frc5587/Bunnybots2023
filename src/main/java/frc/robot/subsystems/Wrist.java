package frc.robot.subsystems;

import frc.robot.Constants.WristConstants;

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
        1,
        0,
        WristConstants.SOFT_LIMITS,
        WristConstants.ZERO_OFFSET,
        WristConstants.ENCODER_CPR,
        WristConstants.PID,
        WristConstants.FF
    );

    public Wrist() {
        this(motor);
    }
    
    public Wrist(CANSparkMax motor) {
        super("wrist", constants, motor);
        enable();
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

    public void wristTop() {
        setGoal(WristConstants.TOP_SETPOINT);
    }

    public void wristMid() {
        setGoal(WristConstants.RESTING_SETPOINT);
    }

    public void wristBottom() {
        setGoal(WristConstants.BOTTOM_SETPOINT);
    }

    @Override
    public void periodic() {
        super.periodic();
        if(SmartDashboard.getBoolean("Reset Encoders", false)) {
            resetEncoders();
        }
        SmartDashboard.putBoolean("Reset Encoders", false);
    }
}
