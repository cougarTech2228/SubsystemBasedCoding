package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.motors.Motor;
import frc.robot.motors.Motor.SetMode;
import frc.robot.motors.Motor.Slot;
import frc.robot.sensors.Pigeon;
import frc.robot.util.Concensus;
import frc.robot.util.Concensus.ConcensusMode;

public class DrivebaseSubsystem extends SubsystemBase {
	private Motor leftMaster;
	private Motor leftFollower;
	private Motor rightMaster;
	private Motor rightFollower;
	private Pigeon pidgey;

	private double maxEncoderCountsPer100ms;

	public static Concensus shouldLowerSpeed = new Concensus(ConcensusMode.Any);

	public DrivebaseSubsystem() {
		leftMaster = Motor.createTalonSRX(0);
		leftFollower = Motor.createTalonSRX(1);
		rightMaster = Motor.createTalonSRX(2);
		rightFollower = Motor.createTalonSRX(3);

		pidgey = new Pigeon(0);

		leftFollower.follow(leftMaster);
		rightFollower.follow(rightMaster);

		leftMaster.invertMotor();
		leftFollower.invertMotor();

		rightMaster.setConversionFactor(1.0 / maxEncoderCountsPer100ms);
		leftMaster.setConversionFactor(1.0 / maxEncoderCountsPer100ms);

		rightMaster.setAuxiliarySource(() -> pidgey.getContinuousYaw() / 180.0);
		leftMaster.setAuxiliarySource(() -> -pidgey.getContinuousYaw() / 180.0);

		rightMaster.setPIDF(Slot.Velocity, 0, 0, 0, 0, 0);
		rightMaster.setPIDF(Slot.Auxiliary, 0, 0, 0, 0, 0);
		rightMaster.setPIDF(Slot.Position, 0, 0, 0, 0, 0);

		leftMaster.setPIDF(Slot.Velocity, 0, 0, 0, 0, 0);
		leftMaster.setPIDF(Slot.Auxiliary, 0, 0, 0, 0, 0);
		leftMaster.setPIDF(Slot.Position, 0, 0, 0, 0, 0);
	}
	@Override
	public void periodic() {
		if(RobotState.isOperatorControl()) {
			double throttle = OI.getXboxLeftJoystickY();
			double turn = OI.getXboxRightJoystickX() * 0.5;

			if(shouldLowerSpeed.getConcensus()) {
				throttle *= 0.5;
				turn *= 0.5;
			}

			rightMaster.set(throttle + turn);
			leftMaster.set(throttle - turn);
		} else if(RobotState.isTest()) {
			double throttle = OI.getXboxLeftJoystickY();

			rightMaster.set(SetMode.Velocity, throttle, 0);
			leftMaster.set(SetMode.Velocity, throttle, 0);
		}
	}
}