// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.subsystems.ElevatorBase.ElevatorConstants;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class ElevConstants {
    public static final boolean motorInverted = false;
    public static final SupplyCurrentLimitConfiguration supplyLimit = new SupplyCurrentLimitConfiguration(true, 40, 35, 1);
    public static final StatorCurrentLimitConfiguration statorLimit = new StatorCurrentLimitConfiguration(true, 40, 35, 1);

    public static final double gearing = 0.1;
    public static final double rtm = 10;
    public static final double[] softLimits = {0, 1};
    public static final int zeroOffset = 10;
    public static final int cpr = 2048;
    public static final int[] switchPort = {0, 1};
    public static final boolean[] switchesInverted = {false, false};
    public static final ProfiledPIDController pid = new ProfiledPIDController(1, 0, 0, 
        new Constraints(1, 1)
    );
    public static final ElevatorFeedforward ff = new ElevatorFeedforward(1, 0, 0, 0);
    public static final ElevatorConstants constants = new ElevatorConstants(gearing, rtm, softLimits, zeroOffset, cpr,
            switchPort, switchesInverted, pid, ff);
  }
}
