// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class CoralSubsystemConstants {
    public static final int kElevatorMotorCanId = 4;
    public static final int kArmMotorCanId = 3;
    public static final int kIntakeMotorCanId = 2;

    public static final class ElevatorSetpoints {
      public static final int kFeederStation = 0;
      public static final int kLevel1 = 0;
      public static final int kLevel2 = 20;
      public static final int kLevel3 = 30;
      public static final int kLevel4 = 40;
    }
    public static final class SimulationRobotConstants {
      public static final double kPixelsPerMeter = 20;
  
      public static final double kElevatorGearing = 20; // 25:1
      public static final double kCarriageMass =
          4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
      public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
      public static final double kMinElevatorHeightMeters = 0.922; // m
      public static final double kMaxElevatorHeightMeters = 1.62; // m
  
      // public static final double kArmReduction = 60; // 60:1
      // public static final double kArmLength = 0.433; // m
      // public static final double kArmMass = 4.3; // Kg
      // public static final double kMinAngleRads =
      //     Units.degreesToRadians(-50.1); // -50.1 deg from horiz
      // public static final double kMaxAngleRads =
      //     Units.degreesToRadians(40.9 + 180); // 40.9 deg from horiz
  
      // public static final double kIntakeReduction = 135; // 135:1
      // public static final double kIntakeLength = 0.4032262; // m
      // public static final double kIntakeMass = 5.8738; // Kg
      // public static final double kIntakeMinAngleRads = Units.degreesToRadians(80);
      // public static final double kIntakeMaxAngleRads = Units.degreesToRadians(180);
      // public static final double kIntakeShortBarLength = 0.1524;
      // public static final double kIntakeLongBarLength = 0.3048;
      // public static final double kIntakeBarAngleRads = Units.degreesToRadians(-60);
    }

  
  }


  

 
  public static final class SimulationRobotConstants {
    public static final double kPixelsPerMeter = 20;

    public static final double kElevatorGearing = 20; // 25:1
    public static final double kCarriageMass =
        4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
    public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
    public static final double kMinElevatorHeightMeters = 0.922; // m
    public static final double kMaxElevatorHeightMeters = 1.62; // m

    public static final double kArmReduction = 60; // 60:1
    public static final double kArmLength = 0.433; // m
    public static final double kArmMass = 4.3; // Kg
    public static final double kMinAngleRads =
        Units.degreesToRadians(-50.1); // -50.1 deg from horiz
    public static final double kMaxAngleRads =
        Units.degreesToRadians(40.9 + 180); // 40.9 deg from horiz

    public static final double kIntakeReduction = 135; // 135:1
    public static final double kIntakeLength = 0.4032262; // m
    public static final double kIntakeMass = 5.8738; // Kg
    public static final double kIntakeMinAngleRads = Units.degreesToRadians(80);
    public static final double kIntakeMaxAngleRads = Units.degreesToRadians(180);
    public static final double kIntakeShortBarLength = 0.1524;
    public static final double kIntakeLongBarLength = 0.3048;
    public static final double kIntakeBarAngleRads = Units.degreesToRadians(-60);
  }
}
