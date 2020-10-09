package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.InterruptHandlerFunction;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.MethodCommand;

public class ClimberSubsystem extends SubsystemBase {

    CANSparkMax climberMotor;
    Solenoid brake, deploy;
    DigitalInput topFlag, bottomFlag;

    public double climbSpeed = 0.05;

    public ClimberSubsystem() {
        register();

        climberMotor = new CANSparkMax(42, MotorType.kBrushless);

        brake = new Solenoid(1);
        deploy = new Solenoid(2);

        topFlag = new DigitalInput(5);
        bottomFlag = new DigitalInput(6);

        climberMotor.setIdleMode(IdleMode.kBrake);

        brake.set(false);

        //Sets up interrupts to immediately stop the elevator when it reaches a flag 
        bottomFlag.requestInterrupts(new InterruptHandlerFunction<Object>() {
            @Override
            public void interruptFired(int interruptAssertedMask, Object param) {
                climberMotor.set(0);
            }
        });
        bottomFlag.setUpSourceEdge(false, true);
        bottomFlag.enableInterrupts();

        topFlag.requestInterrupts(new InterruptHandlerFunction<Object>() {
            @Override
            public void interruptFired(int interruptAssertedMask, Object param) {
                climberMotor.set(0);
            }
        });
        topFlag.setUpSourceEdge(false, true);
        topFlag.enableInterrupts();
    }
    /**
     * Deploys the elevator and limits the shooter and acquisition subsystems
     * while also telling the drivebase to slow
     */
    public Command cmdDeployElevator() {
        return new MethodCommand(() -> {
            deploy.set(true);
            //ShooterSubsystem.shouldShoot.vote(false);
            DrivebaseSubsystem.shouldLowerSpeed.vote(true);
            AcquisitionSubsystem.shouldAcquirerExtend.vote(true);
            AcquisitionSubsystem.shouldAcquire.vote(false);
        });
    }
    /**
     * Retracts the elevator and unlimits other subsystems
     * while resuming the drivebase to normal speed
     */
    public Command cmdRetractElevator() {
        return new MethodCommand(() -> {
            deploy.set(false);
            //ShooterSubsystem.shouldShoot.vote(true);
            DrivebaseSubsystem.shouldLowerSpeed.vote(false);
            AcquisitionSubsystem.shouldAcquirerExtend.vote(false);
            AcquisitionSubsystem.shouldAcquire.vote(true);
        });
    }
    /**
     * Raises the elevator when active, unless it hits a limit switch
     * <p>If the elevator is not deployed, it will deploy it</p>
     */
    public Command cmdRaiseElevator() {
        return cmdDeployElevator()
        .andThen(
            new MethodCommand(() -> {
                brake.set(true);
                climberMotor.set(climbSpeed);
            })
            .runOnEnd(() -> climberMotor.set(0))
            .perpetually()
        );
    }
    /**
     * Lowers the elevator when active, unless it hits a limit switch
     */
    public Command cmdLowerElevator() {
        return new MethodCommand(() -> {
            brake.set(false);
            climberMotor.set(-climbSpeed);
        })
        .runOnEnd(() -> climberMotor.set(0))
        .perpetually();
    }
}