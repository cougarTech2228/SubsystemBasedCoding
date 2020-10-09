package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * The MethodCommand class is meant to be used to easily define commands.
 * It can be used to run any methods it has access to, or define its own to use.
 */
public class MethodCommand extends CommandBase {

    private Runnable method;
    private Runnable endMethod;
    private boolean isFinished;
    /**
     * This constructor should be used when the user wants to run a single method once.
     * @param method = the method to be run
     */
    public MethodCommand(Runnable method) {
        this.method = method;
        this.isFinished = false;
    }
    public MethodCommand() {
        this.method = null;
        this.isFinished = false;
    }
    /**
     * This method is only useful when the command is told to run perpetually,
     * if it is perpetual, when the command is canceled, it will run this method
     */
    public MethodCommand runOnEnd(Runnable method) {
        this.endMethod = method;
        return this;
    }
    /**
     * This method is only useful when the command is told to run perpetually,
     * if it is perpetual, when the command is canceled, it will run this command
     */
    public MethodCommand runOnEnd(Command command) {
        this.endMethod = () -> command.schedule();
        return this;
    }
    @Override
    public void execute() {
        if(method != null) {
            method.run();
        }
        isFinished = true;
    }
    @Override
    public boolean isFinished() {
        return isFinished;
    }
    @Override
    public void end(boolean interrupted) {
        if(endMethod != null) {
            endMethod.run();
        }
    }
}