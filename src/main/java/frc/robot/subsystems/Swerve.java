package frc.robot.subsystems;

import org.frc5587.lib.subsystems.SwerveBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.DrivetrainConstants;

public class Swerve extends SwerveBase {
    private static SwerveModule[] swerveModules = {
            new SwerveModule(DrivetrainConstants.Mod0.MODULE_CONSTANTS, new CANSparkMax(10, MotorType.kBrushless),
                    new CANSparkMax(15, MotorType.kBrushless)),
            new SwerveModule(DrivetrainConstants.Mod1.MODULE_CONSTANTS, new CANSparkMax(11, MotorType.kBrushless),
                    new CANSparkMax(16, MotorType.kBrushless)),
            new SwerveModule(DrivetrainConstants.Mod2.MODULE_CONSTANTS, new CANSparkMax(12, MotorType.kBrushless),
                    new CANSparkMax(17, MotorType.kBrushless)),
            new SwerveModule(DrivetrainConstants.Mod3.MODULE_CONSTANTS, new CANSparkMax(13, MotorType.kBrushless),
                    new CANSparkMax(18, MotorType.kBrushless))
    };

    public Swerve() {
        super(DrivetrainConstants.SWERVE_CONSTANTS, swerveModules);
    }
}
