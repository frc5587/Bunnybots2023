package frc.robot.subsystems;

import org.frc5587.lib.subsystems.ElevatorBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants.ElevConstants;

public class Elevator extends ElevatorBase {
    private static ElevatorConstants constants = ElevConstants.constants;
    private static WPI_TalonFX motor = new WPI_TalonFX(15);

    public Elevator() {
        super(constants, motor);
    }

    @Override
    public double getEncoderPosition() {
        return motor.getSelectedSensorPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return motor.getSelectedSensorVelocity();
    }

    @Override
    public void setEncoderPosition(double position) {
        motor.setSelectedSensorPosition(position);
    }

    @Override
    public void configureMotors() {
        motor.configFactoryDefault();
        motor.setInverted(ElevConstants.motorInverted);
        motor.configSupplyCurrentLimit(ElevConstants.supplyLimit);
        motor.configStatorCurrentLimit(ElevConstants.statorLimit);
    }
}
