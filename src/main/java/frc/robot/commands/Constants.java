// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
	public static final double ROBOT_MASS = 50; // 32lbs * kg per pound
	public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
	public static final double MAX_SPEED = 4.5;
	// Maximum speed of the robot in meters per second, used to limit acceleration.

	public static final class DrivebaseConstants {
		// Hold time on motor brakes when disabled
		public static final double WHEEL_LOCK_TIME = 10; // seconds
	}

	public static class OperatorConstants {
		// Joystick Deadband
		public static final double DEADBAND = 0.1;
		public static final double LEFT_Y_DEADBAND = 0.1;
		public static final double RIGHT_X_DEADBAND = 0.1;
		public static final double TURN_CONSTANT = 6;
		public static final int kDriverControllerPort = 0;
	}

	// Elevator constants
	public static final double ELEVATOR_POSITION_TOLERANCE = 0.2;
	public static final double ELEVATOR_CONVERSION_FACTOR = 25;
	public static final double ELEVATOR_ROLLER_RAIDUS = 3;
	public static final int ELEVATOR_MASTER_MOTOR_ID = 45;
	public static final int ELEVATOR_FOLLOWER_MOTOR_ID = 25;
	public static final double ELEVATOR_MAX_VELO = 3000;
	public static final double ELEVATOR_MAX_ACCELLERATION = 5000;
	public static final double ELEVATOR_P = 0.073;
	public static final double ELEVATOR_I = 0.0;
	public static final double ELEVATOR_D = 0.005;
	public static final int ELEVATOR_CURRENT_LIMIT = 40;
  	public static final double ELEVATOR_MANUAL_POWER = 0.1;
  	public static final double L1_HEIGHT = 0;
  	public static final double L2_HEIGHT = 10.833;
  	public static final double L3_HEIGHT = 26.85;
  	public static final double L4_HEIGHT = 52.16;
  	public static final double CLOSED_HEIGHT = 1;
	public static final double INTAKE_HEIGHT = 5;
	public static final boolean Elevator_INVERTED = true;

	// Auto constants
	public static final double X_REEF_ALIGNMENT_P = 3.3;
	public static final double Y_REEF_ALIGNMENT_P = 3.3;
	public static final double ROT_REEF_ALIGNMENT_P = 0.058;

	public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
	public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
	public static final double X_SETPOINT_REEF_ALIGNMENT = -0.34;  // Vertical pose
	public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
	public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16;  // Horizontal pose
	public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

	public static final double DONT_SEE_TAG_WAIT_TIME = 1;
	public static final double POSE_VALIDATION_TIME = 0.3;

	// Algae intake constants
	public static final int ALGAE_INTAKE_MOTOR_ID = 34;
	public static final int ALGAE_INTAKE_CURRENT_LIMIT = 40;
	public static final double ALGAE_INTAKE_STOP_VELOCITY = 0;
  	public static final double ALGAE_INTAKE_POWER = 0.6;
	public static final int ALGAE_ENCODER_DIO = 9;

	// Algae arm constants
	public static final int ALGAE_ARM_MOTOR_ID = 38;
	public static final double ALGAE_ARM_CONVERSION_FACTOR = 30;
	public static final double ALGAE_ARM_P = 0;
	public static final double ALGAE_ARM_I = 0;
	public static final double ALGAE_ARM_D = 0;
	public static final double ALGAE_ARM_MAX_VEL = 0;
	public static final double ALGAE_ARM_MAX_ACCEL = 0;
	public static final double ALGAE_ARM_TOLERANCE = 0.01;
	public static final double ALGAE_ARM_REEF_POSE = 0.79;
	public static final double ALGAE_ARM_REEF_POSE2 = 0.45;
	public static final double ALGAE_ARM_POSE_IN_FRAME = 0.171;
	public static final double ALGAE_ARM_CLOSED_POSE = 0.2282;  // Out of frame perimeter
	public static final double ALGAE_ARM_OPEN_SPEED = 0.1;
	public static final double ALGAE_ARM_CLOSE_SPEED = -0.04;

	// Coral scores constants
	public static final int CORAL_SCORER_MOTOR_ID = 26;
	public static final int CORAL_SCORER_CURRENT_LIMIT = 110;
	public static final double CORAL_SCORE_POWER = 0.4;
  	public static final double AUTO_CORAL_INTAKE_POWER = 0.05;
	public static final double CORAL_SCORER_MAX_VELO = 3500;
	public static final double CORAL_SCORER_MAX_ACCELLERATION = 12000;
	public static final double CORAL_SCORER_P = 10;
	public static final double CORAL_SCORER_I = 0;
	public static final double CORAL_SCORER_D = 0;
	public static final double CORAL_SCORER_POSITION_TOLERANCE = 0.1;
	public static final double CORAL_SCORER_INTAKE_CURRENT_LIMIT = 50;
	public static final double CORAL_SCORER_INTAKE_ROTS = 0.8;

	// Swerve offsets
	public static final double FRONT_LEFT_ABS_ENCODER_OFFSET = 297.334;
	public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET = 5.801;
	public static final double BACK_LEFT_ABS_ENCODER_OFFSET = 290.215;
	public static final double BACK_RIGHT_ABS_ENCODER_OFFSET = 324.229;
}
