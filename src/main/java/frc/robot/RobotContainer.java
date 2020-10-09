package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.subsystems.AcquisitionSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.DrumSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.CommandToggler;
import frc.robot.util.CommandToggler.CommandState;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  @SuppressWarnings("unused")
  private final static DrivebaseSubsystem m_drivebaseSubsystem = new DrivebaseSubsystem();
  private final static AcquisitionSubsystem m_acquisitionSubsystem = new AcquisitionSubsystem();
  private final static DrumSubsystem m_storageSubsystem = new DrumSubsystem();
  private final static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final static ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  //private final static SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  public RobotContainer() {
    //m_autoChooser.setDefaultOption("Minimum Auto", new MinAutoCommand(m_storageSubsystem, m_shooterSubsystem, m_drivebaseSubsystem));
    configureButtonBindings();
  }

  private void configureButtonBindings() {
      //triggers 
      new Button(OI::getXboxRightTriggerPressed).whenHeld(m_acquisitionSubsystem.cmdAcquireForwards());
      new Button(OI::getXboxLeftTriggerPressed).whenHeld(m_acquisitionSubsystem.cmdAcquireBackwards());

      new CommandToggler( // Acquirer Motor Toggle - Right Bumper
          m_acquisitionSubsystem.cmdDeployAcquirer(),
          m_acquisitionSubsystem.cmdRetractAcquirer()
      )
      .setDefaultState(CommandState.Interruptible)
      .setToggleButton(OI::getXboxRightBumper)
      .setCycle(true);

      //--------------------------------------Shooter Buttons--------------------------------------------------------

      new CommandToggler( // Shooter Motor Toggle - Left Bumper
        m_shooterSubsystem.cmdStartShooter(),
        m_shooterSubsystem.cmdStopShooter()
      )
      .setDefaultState(CommandState.Interruptible)
      .setToggleButton(OI::getXboxLeftBumper)
      .setCycle(true);

      new Button(OI::getXboxAButton).whenHeld(m_storageSubsystem.cmdShootWhileHeld());

      //new Button(OI.getright)

      //--------------------------------------Elevator Buttons-------------------------------------------------------
      
      new CommandToggler( // Deploy/Retract Elevator toggle - start button
        m_climberSubsystem.cmdDeployElevator(),
        m_climberSubsystem.cmdRetractElevator()
      )
      .setDefaultState(CommandState.Interruptible)
      .setToggleButton(OI::getXboxStartButton)
      .setCycle(true);

      new Button(OI::getXboxDpadUp).whenHeld(m_climberSubsystem.cmdRaiseElevator());
      new Button(OI::getXboxDpadDown).whenHeld(m_climberSubsystem.cmdLowerElevator());

      //--------------------------------------Dial Buttons-----------------------------------------------------------
  }
}
