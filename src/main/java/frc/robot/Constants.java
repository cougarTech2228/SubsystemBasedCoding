/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final int CONTROL_PANEL_INTERRUPT_DIO = 0;
	public static final int ACQUIRE_POSITION_DIO = 4;
	public static final int ACQUIRE_BALL_DIO = 2;
	public static final int SHOOTER_BALL_DIO = 3;
	public static final int SHOOTER_POSITION_DIO = 1;
	public static final int CLIMBER_UPPER_PROX_DIO = 5;
	public static final int CLIMBER_LOWER_PROX_DIO = 6;
	public static final int DIGITAL_IO_7 = 7;
	public static final int DIGITAL_IO_8 = 8;
	public static final int DIGITAL_IO_9 = 9;

	public static final int ANALOG_INPUT_0 = 0;
	public static final int ANALOG_INPUT_1 = 1;
	public static final int ANALOG_INPUT_2 = 2;
	public static final int ANALOG_INPUT_3 = 3;

	public static final int BOPPER_PCM_PORT = 0;
	public static final int CLIMBER_BRAKE_PIN_PCM_PORT = 1;
	public static final int CLIMBER_DEPLOY_PCM_PORT = 2;
	public static final int CLIMBER_TRAVERSE_BRAKE_PCM_PORT = 3;
	public static final int ACQUIRER_DEPLOY_PCM_PORT = 4;
	public static final int CONTROL_PANEL_DEPLOY_PCM_PORT = 5;
	public static final int PCM_PORT_6 = 6;
	public static final int PCM_PORT_7 = 7;
	public static final int PCM_CAN_ID = 0;

	public static final int PWM_PIN_0 = 0;
	public static final int PWM_PIN_1 = 1;
	public static final int PWM_PIN_2 = 2;
	public static final int PWM_PIN_3 = 3;
	public static final int PWM_PIN_4 = 4;
	public static final int PWM_PIN_5 = 5;
	public static final int PWM_PIN_6 = 6;
	public static final int PWM_PIN_7 = 7;
	public static final int PWM_PIN_8 = 8;
	public static final int PWM_PIN_9 = 9;

	public static final int CONTROL_PANEL_MOTOR_CAN_ID = 31;
	public static final int RIGHT_FRONT_MOTOR_CAN_ID = 11;
	public static final int RIGHT_REAR_MOTOR_CAN_ID = 12;
	public static final int LEFT_FRONT_MOTOR_CAN_ID = 13;
	public static final int LEFT_REAR_MOTOR_CAN_ID = 14;
	public static final int ACQUISITION_MOTOR_CAN_ID = 21;

	public static final int CLIMBING_NEO_SPARK_MAX_CAN_ID = 42;

	public static final int PIGEON_IMU_CAN_ID = 61;
	public static final int CANIFIER_CAN_ID = 62;

	public static final int LEFT_DISTANCE_SENSOR_ID = 1;
	public static final int RIGHT_DISTANCE_SENSOR_ID = 2;

	public static final int RELAY_PIN_0 = 0;
	public static final int RELAY_PIN_1 = 1;
	public static final int RELAY_PIN_2 = 2;
	public static final int RELAY_PIN_3 = 3;

	public static final double VISION_TARGET_TOLERANCE_IN_INCHES = 1.0;

	public static final double ARCADE_DRIVE_TURN_DEADBAND = .03;

	public static final double XBOX_RUMBLE_SPEED = 1.0;

	public static final double WHEEL_MOTOR_VELOCITY = 0.1;

	public static final int XBOX_RUMBLE_COMMAND_TIMEOUT = 1;

	public static final double CONTROL_PANEL_MOTOR_VELOCITY_FAST = 0.27;
	public static final double CONTROL_PANEL_MOTOR_VELOCITY_SLOW = 0.2;

	public static final double DRUM_MOTOR_VELOCITY = 0.45; 

	public static final int SHOOT_MODE_SINGLE_CELL = 0;
	public static final int SHOOT_MODE_ALL_CELLS = 1;

	public static final int SHOOTER_SLOT = 2;

	public static final int SHOOTER_CAN_ID = 41;
	public static final int DRUM_SPARK_PWM_ID = 0;

	public static final int LOOPS_TO_WAIT = 10;
	public static final double SHOOTER_MOTOR_SPEED = 120000.0;
	public static final int MIN_SHOOTING_DISTANCE = 61;
	public static final int MAX_SHOOTING_DISTANCE = 0;

	public static final double TIME_BETWEEN_SHOTS = 0.5; // 1.0
	public static final double BOPPER_WAIT_TIME = 0.1;

	public static final int DRIVE_CURRENT_LIMIT = 60;
	public static final int DRIVE_CURRENT_DURATION = 100;
	public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 40;

	public static final int ACQUIRE_CURRENT_LIMIT = 30;
	public static final int ACQUIRE_CURRENT_DURATION = 100;
	public static final int ACQUIRE_CONTINUOUS_CURRENT_LIMIT = 18;

	public static final int SHOOTER_CURRENT_LIMIT = 40;
	public static final int SHOOTER_CURRENT_DURATION = 100;
	public static final int SHOOTER_CONTINUOUS_CURRENT_LIMIT = 35;

	public static final int ACQUIRER_CURRENT_THRESHOLD = 2;
	
	public static final double ELEVATOR_DEPLOY_SPEED_LOWER = 0.5;

	/**
	 * Number of joystick buttons to poll. 10 means buttons[1,9] are polled, which
	 * is actually 9 buttons.
	 */
	public final static int kNumButtonsPlusOne = 10;

	/**
	 * How many sensor units per rotation. Using CTRE Magnetic Encoder.
	 * 
	 * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
	 */
	public final static double kEncoderUnitsPerRevolution = 379.16;
	public final static double kWheelDiameterIN = 6.25;
	public final static double kMaxFTPerSecond = 12;
	public final static double kMaxUnitsPer_100ms = 8735.8; // kEncoderUnitsPerRevolution / kWheelDiameterIN / Math.PI *
															// 12 * kMaxFTPerSecond / 10;

	public final static int kSensorUnitsPerRotation = 4096;

	/**
	 * Using the configSelectedFeedbackCoefficient() function, scale units to 3600
	 * per rotation. This is nice as it keeps 0.1 degrees of resolution, and is
	 * fairly intuitive.
	 */
	public final static double kTurnTravelUnitsPerRotation = 3600;

	/**
	 * Empirically measure what the difference between encoders per 360' Drive the
	 * robot in clockwise rotations and measure the units per rotation. Drive the
	 * robot in counter clockwise rotations and measure the units per rotation. Take
	 * the average of the two.
	 */
	public final static int kEncoderUnitsPerRotation = 51711;
	/**
	 * Number of rotations to drive when performing Distance Closed Loop
	 */
	public final static double kRotationsToTravel = 6;

	/**
	 * This is a property of the Pigeon IMU, and should not be changed.
	 */
	public final static int kPigeonUnitsPerRotation = 8192;

	/**
	 * Set to zero to skip waiting for confirmation. Set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public final static int kTimeoutMs = 30;

	/**
	 * Motor neutral dead-band, set to the minimum 0.1%.
	 */
	public final static double kNeutralDeadband = 0.001;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control
	 * loop. kF: 1023 represents output value to Talon at 100%, 6800 represents
	 * Velocity units at 100% output Not all set of Gains are used in this project
	 * and may be removed as desired.
	 * 
	 * kP kI kD kF Iz PeakOut
	 */
	// public final static Gains kGains_Distanc = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.50);
	// public final static Gains kGains_Turning = new Gains(2.0, 0.0, 4.0, 0.0, 200, 1.00);
	// public final static Gains kGains_Velocit = new Gains(0.1, 0.0, 20.0, 1023.0 / 6800.0, 300, 0.50);
	// public final static Gains kGains_MotProf = new Gains(1.0, 0.0, 0.0, 1023.0 / 6800.0, 400, 1.00);

	/** ---- Flat constants, you should not need to change these ---- */
	/*
	 * We allow either a 0 or 1 when selecting an ordinal for remote devices [You
	 * can have up to 2 devices assigned remotely to a talon/victor]
	 */
	public final static int REMOTE_0 = 0;
	public final static int REMOTE_1 = 1;
	/*
	 * We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1
	 * is auxiliary
	 */
	public final static int PID_PRIMARY = 0;
	public final static int PID_TURN = 1;
	/*
	 * Firmware currently supports slots [0, 3] and can be used for either PID Set
	 */
	public final static int SLOT_0 = 0;
	public final static int SLOT_1 = 1;
	public final static int SLOT_2 = 2;
	public final static int SLOT_3 = 3;
	/* ---- Named slots, used to clarify code ---- */
	public final static int kSlot_Distanc = SLOT_0;
	public final static int kSlot_Turning = SLOT_1;
	public final static int kSlot_Velocit = SLOT_2;
	public final static int kSlot_MotProf = SLOT_3;

    // Stuff needed for Auto Ramsete-based Trajectories

	// Need to set this to TRUE for Ramsete Commands but it
	// seems to give the negated value not the correct ones
	// when we're trying to manually create autonomous paths
	public final static boolean kGyroReversed = true;

	public static final int EDGES_PER_ROTATION = 512;
	public static final double WHEEL_DIAMETER_INCHES = 8.0; // TODO 4 on Pizza Box, change to 8 on Xi
    public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;

    public static final double TRACK_WIDTH_METERS = .590; //0.647;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
			TRACK_WIDTH_METERS);

	// Baseline values for a RAMSETE follower in units of meters and seconds
	public static final double RAMSETE_B = 2;
	public static final double RAMSETE_ZETA = 0.7;

	// Voltage needed to overcome the motors static friction. kS 
	public static final double kS = 0.923; //PB value = 1.55;

	// Voltage needed to hold (or "cruise") at a given constant velocity. kV
	public static final double kV = 1.42; //PB value = 2.73; 

	// Voltage needed to induce a given acceleration in the motor shaft. kA 
	public static final double kA = 0.414; //PB value = 0.184;

	public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(kS, kV, kA);
	
	public static final double kMaxSpeedMetersPerSecond = 1.5;
	public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;

	public static final double DIFFERENTIAL_DRIVE_CONSTRAINT_MAX_VOLTAGE = 10.0;

	public static final double ACQUIRER_MOTOR_SPEED = 0.70;

}