package frc.robot.util;

import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A class dedicated to make it easy to make buttons toggle between or through multiple commands
 */
public class CommandToggler {

    private boolean cycle;
    private List<Command> commands;
    private List<CommandState> commandStates;
    private Button upButton, downButton;
    private int currentIndex;
    private CommandState defaultCmdState = CommandState.Normal;

    /**
     * CommandStates define how the commands should be run in the CommandToggler's command list
     */
    public enum CommandState {
        /**
         * Normal mode means that this command can run simultaneously with the next command
         */
        Normal,
        /**
         * Interruptible mode means that when the next command is run, this one stops
         */
        Interruptible,
        /**
         * Precendented mode means that this command will take precedence over the next command,
         * it will have to finish before the next command can be run
         */
        Precedented
    }
    /**
     * Creates a new default CommandToggler instance
     */
    public CommandToggler() {
        this.commands = new ArrayList<Command>();
        this.commandStates = new ArrayList<CommandState>();
        currentIndex = -1;
        cycle = false;
    }
    /**
     * Creates a new CommandToggler that toggles between the commands passed in
     * @param commands
     */
    public CommandToggler(Command... commands) {
        this.commands = Arrays.asList(commands);
        this.commandStates = new ArrayList<CommandState>();
        setDefaultState(CommandState.Normal);
        currentIndex = -1;
        cycle = false;
    }
    /**
     * Adds the specified command onto the toggling list with the specified CommandState
     * @param command
     * @param state
     */
    public CommandToggler addCommand(Command command, CommandState state) {
        commands.add(command);
        commandStates.add(state);
        return this;
    }
    /**
     * Adds the specified command onto the toggling list with the specified CommandState
     * but when this command finishes, it will automatically jump to the next command and run it
     * @param command
     * @param state
     */
    public CommandToggler addJumpCommand(Command command, CommandState state) {
        addCommand(command.andThen(() -> runIndex(currentIndex + 1)), state);
        return this;
    }
    /**
     * Adds the specified command onto the toggling list with the default CommandState
     * but when this command finishes, it will automatically jump to the next command and run it
     * @param command
     */
    public CommandToggler addJumpCommand(Command command) {
        addJumpCommand(command, defaultCmdState);
        return this;
    }
    /**
     * Adds the specified command onto the toggling list with the default CommandState
     * @param command
     */
    public CommandToggler addCommand(Command command) {
        return addCommand(command, defaultCmdState);
    }
    /**
     * Sets the default CommandState, *This must be run before any calls to addCommand()
     * @param state
     * @return
     */
    @SuppressWarnings("all")
    public CommandToggler setDefaultState(CommandState state) {
        commandStates = new ArrayList<CommandState>();
        for(Command c : commands) {
            commandStates.add(state);
        }
        return this;
    }
    /**
     * When the robot is enabled, it will jump to running the first command
     */
    public CommandToggler startOnEnable() {
        new Button(RobotState::isEnabled).whenActive(() -> {
            System.out.println("enabled");
            runIndex(0);
        });
        return this;
    }
    /**
     * when the condition passed in is true, the CommandList will go to the specified command,
     * unless it is not able to run, ie: in the case of having a precendented command still running
     * @param index the index of the command to jump to
     * @param condition the condition that must be fulfilled
     */
    public CommandToggler jumpTo(int index, BooleanSupplier condition) {
        new Trigger(condition).whenActive(() -> {
            runIndex(index);
            System.out.println("GAMMER");
        });
        return this;
    }
    /**
     * Sets whether to cycle through the command list, if true, once the last command has been started,
     * the next time the set CommandToggler is pressed, it will loop to the beginning of the command list and run the first command
     * (The exact opposite happens for the ToggleDownButton if one is assigned)
     * @param on
     */
    public CommandToggler setCycle(boolean on) {
        cycle = on;
        return this;
    }
    /**
     * Assigns a button to control toggling upward through the command list
     * @param pressed = a reference to the method testing the button press
     * Ex: .setCommandToggler(OI::getXboxLeftBumper)
     */
    public CommandToggler setToggleButton(BooleanSupplier pressed) {
        return setToggleButton(new Button(pressed));
    }

    private void runIndex(int index) {
        if (index > commands.size() - 1) {
            index = cycle ? 0 : currentIndex;
        }
        else if (index < 0) {
            index = cycle ? commands.size() - 1 : currentIndex;
        }
        if(currentIndex != index) {
            if(currentIndex != -1) {
                CommandState state = commandStates.get(currentIndex);
                if(state == CommandState.Precedented && !commands.get(currentIndex).isFinished()) {
                    return;
                }
                if(state == CommandState.Interruptible && commands.get(currentIndex) != null) {
                    commands.get(currentIndex).cancel();
                }
            }
            if(commands.get(index) != null) {
                CommandScheduler.getInstance().schedule(commands.get(index));
            }
            currentIndex = index;
        }
    }
    /**
     * Assigns a button to control toggling upward through the command list
     * @param button = a Button object
     */
    public CommandToggler setToggleButton(Button button) {
        
        if(upButton == null) {
            upButton = button;
            upButton.whenPressed(() -> {
                runIndex(currentIndex + 1);
            });
        }
        /*
        if(upButton == null) {
            upButton = new Button(() -> button.get() || (start && RobotState.isEnabled()));
            upButton.whenPressed(() -> {
                start = false;
                if(currentIndex != -1) {
                    CommandState state = commandStates.get(currentIndex);
                    if(state == CommandState.Precedented && !commands.get(currentIndex).isFinished()) {
                        return;
                    }
                    if(state == CommandState.Interruptible && commands.get(currentIndex) != null) {
                        commands.get(currentIndex).cancel();
                    }
                }
                if(commands.get(nextIndex) != null) {
                    CommandScheduler.getInstance().schedule(commands.get(nextIndex));
                }
                currentIndex = nextIndex;

                nextIndex++;
                if(nextIndex > commands.size() - 1) nextIndex = cycle ? 0 : currentIndex;
            });
        }*/
        return this;
    }
    /**
     * Assigns a button to control toggling upward through the command list
     * @param pressed = a reference to the method testing the button press
     * Ex: .setToggleDownButton(OI::getXboxRightBumper)
     */
    public CommandToggler setToggleDownButton(BooleanSupplier pressed) {
        return setToggleDownButton(new Button(pressed));
    }
    /**
     * Assigns a button to control toggling downward through the command list
     * @param button = a Button object
     */
    public CommandToggler setToggleDownButton(Button button) {
        if(downButton == null) {
            downButton = button;
            downButton.whenPressed(() -> {
                runIndex(currentIndex - 1);
            });
            /*
            downButton.whenPressed(() -> {
                if(currentIndex != -1) {
                    CommandState state = commandStates.get(currentIndex);
                    if(state == CommandState.Precedented && !commands.get(currentIndex).isFinished()) {
                        return;
                    }
                    if(state == CommandState.Interruptible) {
                        commands.get(currentIndex).cancel();
                    }
                }
                if(commands.get(nextIndex) != null) {
                    CommandScheduler.getInstance().schedule(commands.get(nextIndex));
                }
                currentIndex = nextIndex;

                nextIndex--;
                if(nextIndex < 0) nextIndex = cycle ? commands.size() - 1 : currentIndex;
            });*/
        }
        return this;
    }
    /**
     * Returns the command that is currently being run
     */
    public Command getActiveCommand() {
        return commands.get(currentIndex);
    }
    /**
     * returns the index of the command that is currently being run
     */
    public int getCommandIndex() {
        return currentIndex;
    }
}