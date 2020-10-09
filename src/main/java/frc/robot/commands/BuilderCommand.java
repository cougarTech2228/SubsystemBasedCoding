package frc.robot.commands;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * The ContinuousCommand class is meant to be used to easily define commands with complex logic
 */
public class BuilderCommand extends CommandBase {
    private Controller controller;
    private Function<Controller, Controller> conditions;
    private boolean isFinished;
    private boolean hasStarted;
    private boolean cancelOnFinished;
    private Command runOnEnd;

    /**
     * define a ContinuousCommand with the specified conditions
     * <p>Note: all commands that are run in a single iteration are executed parallelly to eachother</p>
     */
    public BuilderCommand(Function<Controller, Controller> conditions) {
        this.conditions = conditions;
        this.isFinished = false;
        this.hasStarted = false;
        this.cancelOnFinished = false;
    }
    private void startNext() {
        if(!controller.isFinished) {
            controller.getActiveCommand().andThen(() -> {
                if(controller.afterwardStop || isFinished) {
                    isFinished = true;
                } else {
                    controller = conditions.apply(new Controller(controller.iteration + 1));
                    startNext();
                }
            }).schedule();
        }
        else {
            isFinished = true;
        }
    }
    @Override
    public void initialize() {
        hasStarted = false;
        isFinished = false;
    }
    @Override
    public void execute() {
        if(!hasStarted) {
            controller = conditions.apply(new Controller(0));
            startNext();
            hasStarted = true;
        }
    }
    public BuilderCommand cancelOnFinished() {
        cancelOnFinished = true;
        return this;
    }
    @Override
    public boolean isFinished() {
        return isFinished;
    }
    @Override
    public void end(boolean interrupted) {
        if(cancelOnFinished) {
            controller.getActiveCommand().cancel();
        }
        if(runOnEnd != null) {
            runOnEnd.schedule();
        }
    }
    public BuilderCommand runOnEnd(Command c) {
        runOnEnd = c;
        return this;
    }
    public BuilderCommand runOnEnd(Runnable r) {
        runOnEnd = new MethodCommand(r);
        return this;
    }
    /**
     * The Controller has a crap ton of options to very quickly and easily define complex logic for commands
     */
    public final class Controller {
        private int iteration;
        private Boolean when;
        private boolean wasJustTrue;
        private Command activeCommand;
        private boolean isFinished;
        private boolean afterwardStop;
        private Controller(int iteration) {
            this.iteration = iteration;
            this.when = null;
            this.isFinished = false;
            this.wasJustTrue = false;
            this.afterwardStop = false;
        }
        /**
         * gets how many times this controller has been checked
         */
        public int getIteration() {
            return iteration;
        }
        /**
         * ends if the previous condition returned true
         */
        public Controller endAfterward() {
            if(wasJustTrue) afterwardStop = true;
            return this;
        }
        /**
         * ends if the condition is true
         */
        public Controller endWhen(boolean b) {
            wasJustTrue = false;
            if(b) isFinished = true;
            return this;
        }
        /**
         * ends on the specified iteration. It skips running the commands
         */
        public Controller endOn(int iteration) {
            wasJustTrue = false;
            if(this.iteration == iteration) isFinished = true;
            return this;
        }
        /**
         * ends after the specified iteration
         */
        public Controller endAfter(int iteration) {
            wasJustTrue = false;
            if(this.iteration == iteration + 1) isFinished = true;
            return this;
        }
        /**
         * when the condition is true, the command is scheduled to run
         */
        public Controller when(boolean condition, Command command) {
            wasJustTrue = false;
            if(condition) {
                addCommand(command);
                wasJustTrue = true;
            }
            when = condition;
            return this;
        }
        /**
         * when the condition is true, the method is scheduled to run
         */
        public Controller when(boolean condition, Runnable method) {
            return when(condition, new MethodCommand(method));
        }
        /**
         * if the previous condition is false, and this one is true, the command is scheduled to run
         */
        public Controller otherwiseWhen(boolean condition, Command command) {
            wasJustTrue = false;
            if(when == null) {
                //ahh
            }
            else if(!when && condition) {
                addCommand(command);
                wasJustTrue = true;
            }
            when = when || condition;
            return this;
        }
        /**
         * if the previous condition is false, and this one is true, the method is scheduled to run
         */
        public Controller otherwiseWhen(boolean condition, Runnable method) {
            return otherwiseWhen(condition, new MethodCommand(method));
        }
        /**
         * if all previous conditions are false, this command is scheduled to run
         */
        public Controller otherwise(Command command) {
            wasJustTrue = false;
            if(when == null) {
                //ahh
            }
            else if(!when) {
                addCommand(command);
                wasJustTrue = true;
            }
            when = null;
            return this;
        }
        /**
         * if all previous conditions are false, this method is scheduled to run
         */
        public Controller otherwise(Runnable method) {
            return otherwise(new MethodCommand(method));
        }
        /**
         * this command will run each iteration, unconditionally
         */
        public Controller run(Command command) {
            wasJustTrue = false;
            addCommand(command);
            return this;
        }
        /**
         * this method will run once each iteration, unconditionally
         */
        public Controller run(Runnable method) {
            return run(new MethodCommand(method));
        }
        private void addCommand(Command command) {
            if(activeCommand == null) {
                activeCommand = command;
            } else {
                activeCommand = activeCommand.alongWith(command);
            }
        }
        private Command getActiveCommand() {
            if(activeCommand == null)
                return new MethodCommand();
            return activeCommand;
        }
    }
}