package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.MethodCommand;
import frc.robot.motors.Motor;
import frc.robot.motors.Motor.SetMode;
import frc.robot.motors.Motor.Slot;
import frc.robot.sensors.GarminLidar;
import frc.robot.util.Concensus;
import frc.robot.util.Concensus.ConcensusMode;

public class ShooterSubsystem extends SubsystemBase {
    private boolean isRunning;
    private HashMap<Integer, Double> shooterMap;
    private Motor shooterMotor;
    private GarminLidar lidarSensor;

    public static final Concensus shouldShoot = new Concensus(ConcensusMode.All);

    public ShooterSubsystem() {
        shooterMotor = Motor.createTalonSRX(Constants.SHOOTER_CAN_ID);

        shooterMap = new HashMap<Integer, Double>();
        lidarSensor = new GarminLidar();
        
        shooterMap.put(80, 110512.0);
        shooterMap.put(81, 106598.0);
        shooterMap.put(83, 100577.0);
        shooterMap.put(84, 98206.9);
        shooterMap.put(87, 92757.3);
        shooterMap.put(90, 88939.9);
        shooterMap.put(95, 84649.9);
        shooterMap.put(100, 81867.3);
        shooterMap.put(105, 79972.3);
        shooterMap.put(110, 78646.6);
        shooterMap.put(120, 77046.0);
        shooterMap.put(130, 76280.6);
        shooterMap.put(140, 75999.4);
        shooterMap.put(150, 76021.1);
        shooterMap.put(160, 76242.2);
        shooterMap.put(170, 76599.9);
        shooterMap.put(180, 77053.6);
        shooterMap.put(190, 77576.1);
        shooterMap.put(200, 78148.6);
        shooterMap.put(210, 78757.7);
        shooterMap.put(220, 79393.6);
        shooterMap.put(230, 80049.2);
        shooterMap.put(240, 80719.0);
        shooterMap.put(250, 81399.0);
        shooterMap.put(260, 82085.8);
        shooterMap.put(270, 82777.1);
        shooterMap.put(280, 83470.9);
        shooterMap.put(290, 84165.7);
        shooterMap.put(300, 84860.4);
        shooterMap.put(310, 85553.8);
        shooterMap.put(320, 86245.3);
        shooterMap.put(330, 86934.3);
        shooterMap.put(340, 87620.3);
        shooterMap.put(350, 88302.9);
        shooterMap.put(360, 88981.8);
        
        shooterMotor.setPIDF(Slot.Velocity, 0.01764, 0, 0, 0.00851, 0);
        shooterMotor.setVoltageCompensation(11);
        shooterMotor.invertMotor();
        shooterMotor.setCurrentLimit(Constants.SHOOTER_CURRENT_LIMIT);
    }
    public double correspondingSpeed(int distance) {
        int closestDistance = 80;
        for(int i : shooterMap.keySet()) {
            if(distance >= closestDistance)
                closestDistance = i;
            else break;
        }
        return shooterMap.get(closestDistance) - 1500;
    }
    @Override
    public void periodic() {
        if(isRunning && shouldShoot.getConcensus()) {
            shooterMotor.set(SetMode.Velocity, correspondingSpeed(lidarSensor.getAverage()));
        } else {
            shooterMotor.set(SetMode.PercentOutput, 0);
        }
    }
    public Command cmdStartShooter() {
        return new MethodCommand(() -> {
            isRunning = true;
            AcquisitionSubsystem.shouldAcquirerExtend.vote(true);
            AcquisitionSubsystem.shouldAcquire.vote(false);
        });
    }
    public boolean isShooterMotorRunning() {
        return isRunning && shouldShoot.getConcensus();
    }
    public boolean isShooterMotorActivated() {
        return isRunning;
    }
    public Command cmdStopShooter() {
        return new MethodCommand(() -> {
            isRunning = false;
            AcquisitionSubsystem.shouldAcquirerExtend.vote(false);
            AcquisitionSubsystem.shouldAcquire.vote(true);
        });
    }
}