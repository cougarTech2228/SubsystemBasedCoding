package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.BuilderCommand;

public class DrumSubsystem extends SubsystemBase {

    private DigitalInput m_inputBallChecker;
    private DigitalInput m_inputIndexChecker;
    private DigitalInput m_shootBallChecker;
    private DigitalInput m_shootIndexChecker;
    private boolean isShooting = false;
    private Spark m_drumSparkMotor;
    private Solenoid bopper;

    public enum StorageState {
        Shooting(true),
        Acquiring(false);

        boolean value;
        StorageState(boolean value) {
            this.value = value;
        }
    }

    public DrumSubsystem() {

        // You need to register the subsystem to get it's periodic
        // method to be called by the Scheduler
        register();
        m_inputBallChecker = new DigitalInput(Constants.ACQUIRE_BALL_DIO);
        m_inputIndexChecker = new DigitalInput(Constants.ACQUIRE_POSITION_DIO);
        m_shootBallChecker = new DigitalInput(Constants.SHOOTER_BALL_DIO);
        m_shootIndexChecker = new DigitalInput(Constants.SHOOTER_POSITION_DIO);
        // m_drumMotor = new TalonSRXMotor(-1s);
        m_drumSparkMotor = new Spark(Constants.DRUM_SPARK_PWM_ID);

        bopper = new Solenoid(Constants.BOPPER_PCM_PORT);

        new Trigger(() -> !m_inputBallChecker.get()).whenActive(cmdRotateDrumOnce());
    }

    @Override
    public void periodic() {

    }
    /**
     * This command shoots repeatedly while holding a button
     */
    public Command cmdShootWhileHeld() {
        return new BuilderCommand(c -> c
            .run(cmdTryShootOnce())
        ).runOnEnd(cmdSetMode(StorageState.Acquiring));
    }
    /**
     * Spins drum until a ball located, then shoots
     */
    public Command cmdShootNext() {
        return cmdSetMode(StorageState.Shooting)
        .andThen(
            new BuilderCommand(c -> c
                .when(!m_shootBallChecker.get(), cmdBop().andThen(cmdRotateDrumOnce()))
                .endAfterward()
                .otherwise(new WaitCommand(0.04).andThen(cmdRotateDrumOnce()))
                .endAfter(5)
            ),
            cmdSetMode(StorageState.Acquiring)
        );
    }
    /**
     * Spins the drum one full revolution, while shooting all cells
     */
    public Command cmdShootAll() {
        return new SequentialCommandGroup(
            cmdSetMode(StorageState.Shooting),
            new BuilderCommand(c -> c
                .run(cmdTryShootOnce())
                .endAfter(5)
            ),
            cmdSetMode(StorageState.Acquiring)
        );
    }
    /**
     * Sets the storage mode (shooting or acquiring) and rotates the drum respectively
     */
    public Command cmdSetMode(StorageState state) {        
        return new BuilderCommand(c -> c
            .when(isShooting != state.value, cmdRotateDrumOnce().beforeStarting(() -> isShooting = state.value))
            .endOn(0)
        );
    }
    /**
     * Rotate the drum 1 index based on shooting mode
     */
    public Command cmdRotateDrumOnce() {
        return new WaitCommand(0.2)
            .beforeStarting(() -> m_drumSparkMotor.set(0.4))
            .andThen(
                new BuilderCommand(c -> c
                    .when(isShooting ? m_shootIndexChecker.get() : m_inputIndexChecker.get(), () -> m_drumSparkMotor.set(0))
                    .endAfterward()
                )
            );
    }
    /**
     * If a ball is loaded, shoot it, then rotate
     */
    public Command cmdTryShootOnce() {
        return new SequentialCommandGroup(
            cmdSetMode(StorageState.Shooting),
            new BuilderCommand(c -> c
                .when(!m_shootBallChecker.get(), cmdBop())
                .endAfter(0)
            ),
            cmdRotateDrumOnce()
        );
    }
    /**
     * run the solenoid on/off quickly to bop a ball into the shooter
     */
    public Command cmdBop() {
        return new WaitCommand(0.1)
            .beforeStarting(() -> bopper.set(true))
            .andThen(() -> bopper.set(false));
    }
}