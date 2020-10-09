package frc.robot.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.BuilderCommand;

import java.util.LinkedList;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Motor {
    // setup background thread for PID control
    private static int periodMs = 2;
    private static LinkedList<Motor> allMotors = new LinkedList<Motor>();

    private static class MotorThread {
        Thread thread;
        public MotorThread() {
            thread = new Thread(() -> {
                while (true) {
                    for (Motor m : allMotors) {
                        m.update();
                    }
                    try {
                        Thread.sleep(periodMs);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            });
            thread.start();
        }
    }
    @SuppressWarnings({"unused"})
    private static MotorThread motorUpdateThread = new MotorThread();

    // private objects to get/set motor data without using a specific motor
    private Supplier<Double> getEncoderPos;
    private Supplier<Double> getEncoderVel;
    private Supplier<Double> getCurrent;

    private Consumer<Double> setSpeed;
    private Consumer<BrakeMode> setBrakeMode;
    private Consumer<Integer> setCurrentLimit;
    private Consumer<Double> setVoltageCompensation;
    
    private Runnable resetEncoder;

    private double[] rollingAvg;
    private int rollingAvgIndex;

    // used for PID calculation
    private double prevError;
    private double integral;

    private double prevAuxError;
    private double auxIntegral;

    private double setPoint;
    private double auxSetPoint;

    private double output;

    private Supplier<Double> sensorSource;
    private Supplier<Double> auxSensorSource;

    // motor characteristics

    // PID 0-position, 1-velocity, 2-other, 3-auxilary
    private double[] kP, kI, kD, kFF, kIzone;
    //private int activePID;
    private boolean useAux;

    private boolean isEncoderInverted;
    private boolean isMotorInverted;

    private double conversionFactor = 1;

    private Motor master;

    private SetMode mode = SetMode.PercentOutput;

    public enum Slot {
        Position(0), Velocity(1), Sens(2), Auxiliary(3);

        private int slot;

        private Slot(int slot) {
            this.slot = slot;
        }
    }

    // PID mode
    public enum SetMode {
        Position(0), Velocity(1), Sensor(2), PercentOutput(-1), Follow(-1);

        private int slot;

        private SetMode(int slot) {
            this.slot = slot;
        }
    }

    private enum BrakeMode {
        Brake(true), Coast(false);

        private boolean value;

        private BrakeMode(boolean value) {
            this.value = value;
        }
    }

    private Motor() {
        kP = new double[4];
        kI = new double[4];
        kD = new double[4];
        kFF = new double[4];
        kIzone = new double[4];
        rollingAvg = new double[3];
    }

    private void init() {
        allMotors.add(this);
    }

    // static 'constructors'
    public static Motor createTalonSRX(int port) {
        TalonSRX talon = new TalonSRX(port);
        talon.configFactoryDefault();
        Motor ret = new Motor();
        ret.getEncoderPos = () -> ret.conversionFactor * talon.getSelectedSensorPosition();
        ret.getEncoderVel = () -> ret.conversionFactor * talon.getSelectedSensorVelocity();
        ret.setSpeed = p -> talon.set(ControlMode.PercentOutput, p);
        ret.setBrakeMode = brakeMode -> talon.setNeutralMode(brakeMode.value ? NeutralMode.Brake : NeutralMode.Coast);
        ret.getCurrent = () -> talon.getStatorCurrent();
        ret.setCurrentLimit = limit -> {
            talon.configContinuousCurrentLimit(limit);
            talon.enableCurrentLimit(true);
        };
        ret.setVoltageCompensation = voltage -> {
            talon.configVoltageCompSaturation(voltage);
            talon.enableVoltageCompensation(true);
        };
        ret.init();
        return ret;
    }

    public static Motor createFalcon500(int port) {
        TalonFX talon = new TalonFX(port);
        talon.configFactoryDefault();
        Motor ret = new Motor();
        ret.getEncoderPos = () -> ret.conversionFactor * talon.getSelectedSensorPosition();
        ret.getEncoderVel = () -> ret.conversionFactor * talon.getSelectedSensorVelocity();
        ret.setSpeed = p -> talon.set(ControlMode.PercentOutput, p);
        ret.setBrakeMode = brakeMode -> talon.setNeutralMode(brakeMode.value ? NeutralMode.Brake : NeutralMode.Coast);
        ret.getCurrent = () -> talon.getStatorCurrent();
        ret.resetEncoder = () -> talon.setSelectedSensorPosition(0);
        ret.setCurrentLimit = limit -> {
            talon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, limit, 0, 1));
        };
        ret.setVoltageCompensation = voltage -> {
            talon.configVoltageCompSaturation(voltage);
            talon.enableVoltageCompensation(true);
        };
        ret.init();
        return ret;
    }

    public static Motor createVenom(int port) {
        @SuppressWarnings({"resource"})
        CANVenom venom = new CANVenom(port);
        Motor ret = new Motor();
        ret.getEncoderPos = () -> ret.conversionFactor * venom.getPosition();
        ret.getEncoderVel = () -> ret.conversionFactor * venom.getSpeed();
        ret.setSpeed = p -> venom.set(p);
        ret.setBrakeMode = brakeMode -> venom
                .setBrakeCoastMode(brakeMode.value ? BrakeCoastMode.Brake : BrakeCoastMode.Coast);
        ret.getCurrent = () -> venom.getOutputCurrent();
        ret.setCurrentLimit = limit -> DriverStation.reportError("Venom motors do not support current limiting", true);
        ret.setVoltageCompensation = voltage -> DriverStation
                .reportError("Venom motors do not support voltage compensation", true);
        ret.init();
        return ret;
    }

    public static Motor createNeo(int port) {
        @SuppressWarnings({"resource"})
        CANSparkMax neo = new CANSparkMax(port, MotorType.kBrushless);
        neo.restoreFactoryDefaults();
        Motor ret = new Motor();
        ret.getEncoderPos = () -> neo.getEncoder().getPosition();
        ret.getEncoderVel = () -> neo.getEncoder().getVelocity();
        ret.setSpeed = p -> neo.set(p);
        ret.setBrakeMode = brakeMode -> neo.setIdleMode(brakeMode.value ? IdleMode.kBrake : IdleMode.kCoast);
        ret.getCurrent = () -> neo.getOutputCurrent();
        ret.setCurrentLimit = limit -> neo.setSmartCurrentLimit(limit);
        ret.setVoltageCompensation = voltage -> neo.enableVoltageCompensation(voltage);
        ret.init();
        return ret;
    }

    /**
     * Sets the PID values, and the FeedForward coefficient
     */
    public void setPIDF(Slot slot, double kP, double kI, double kD, double kFF, double kIzone) {
        this.kP[slot.slot] = kP;
        this.kI[slot.slot] = kI;
        this.kD[slot.slot] = kD;
        this.kFF[slot.slot] = kFF;
        this.kIzone[slot.slot] = kIzone;
    }

    /**
     * Sets the PID values
     */
    public void setPID(Slot slot, double kP, double kI, double kD) {
        setPIDF(slot, kP, kI, kD, kFF[slot.slot], kIzone[slot.slot]);
    }

    public void resetEncoder() {
        resetEncoder.run();
    }

    /**
     * This allows you to set a conversion factor to convert native units into the
     * desired units
     * <p>
     * Ex: setEncoderScale(inPerNativeUnits)
     * </p>
     * 
     * @param conversionFactor
     */
    public void setConversionFactor(double conversionFactor) {
        this.conversionFactor = conversionFactor;
    }

    /**
     * This allows you to totally customize how what the target is checked, opening
     * up alot of possibilities. Like driving straight
     * 
     * @param targetValue
     * @param currentPoint
     */

    public void setSensorSource(Supplier<Double> value) {
        this.sensorSource = value;
    }

    public void setAuxiliarySource(Supplier<Double> value) {
        this.auxSensorSource = value;
    }

    /**
     * Gets the closed loop error
     */
    public double getError() {
        return prevError;
    }

    /**
     * Gets the output current
     */
    public double getCurrent() {
        return getCurrent.get();
    }

    /**
     * Sets a current limit. Note: this function is not available for Venoms
     */
    public void setCurrentLimit(int limit) {
        this.setCurrentLimit.accept(limit);
    }

    /**
     * Sets voltage compensation. Note: this function is not available for Venoms
     */
    public void setVoltageCompensation(double voltage) {
        this.setVoltageCompensation.accept(voltage);
    }

    /**
     * Inverts the encoder
     */
    public void invertEncoder() {
        isEncoderInverted = !isEncoderInverted;
        conversionFactor = -conversionFactor;
    }

    /**
     * Inverts the motor output (Does not affect any follower output)
     */
    public void invertMotor() {
        isMotorInverted = !isMotorInverted;
    }

    public double getVelocity() {
        return getEncoderVel.get();
    }

    public double getPosition() {
        return getEncoderPos.get();
    }

    /**
     * This motor matches the output of its master. This is not affected when the
     * master's output is inverted
     */
    public void follow(Motor master) {
        mode = SetMode.Follow;
        this.master = master;

        // make sure the follower motor is updated after its master by putting it at the
        // end of the list
        allMotors.remove(this);
        allMotors.addLast(this);
    }

    /**
     * Sets the idleMode: Brake or Coast
     */
    public void setIdleMode(BrakeMode mode) {
        setBrakeMode.accept(mode);
    }

    /**
     * Sets the percentOutput between -1 and 1
     */
    public void set(double percentOutput) {
        set(SetMode.PercentOutput, percentOutput);
    }

    public void set(SetMode mode, double targetValue) {
        if(this.mode != mode) {
            resetAllPID();
        }
        this.useAux = false;
        this.mode = mode;
        this.setPoint = targetValue;
    }

    public void set(SetMode mode, double targetValue, double auxTargetValue) {
        if(this.mode != mode || !useAux) {
            resetAllPID();
        }
        this.mode = mode;
        this.setPoint = targetValue;
        this.useAux = true;
        this.auxSetPoint = auxTargetValue;
    }

    public void resetAllPID() {
        prevAuxError = 0;
        auxIntegral = 0;
        prevError = 0;
        integral = 0;
    }

    /**
     * Does PID calculations, follows a master, or sets percentOutput
     */
    private void update() {
        double error = 0;
        switch (mode) {
        case Follow:
            output = master.output;
            setSpeed.accept(isMotorInverted ? -output : output);
            return;
        case PercentOutput:
            output = setPoint;
            setSpeed.accept(isMotorInverted ? -output : output);
            return;
        case Position:
            error = setPoint - getEncoderPos.get();
            break;
        case Velocity:
            error = setPoint - getEncoderVel.get();
            break;
        case Sensor:
            error = setPoint - sensorSource.get();
            break;
        }
        rollingAvg[rollingAvgIndex] = error;
        rollingAvgIndex++;
        if(rollingAvgIndex >= rollingAvg.length) rollingAvgIndex = 0;
        error = 0;
        for(double d : rollingAvg) {
            error += d;
        }
        error /= rollingAvg.length;

        if (Math.abs(integral) < kIzone[mode.slot])
            integral += error * periodMs;

        double derivative = (error - prevError) / periodMs;
        output = kP[mode.slot] * error + kI[mode.slot] * integral + kD[mode.slot] * derivative
                + setPoint * kFF[mode.slot];

        if (useAux) {
            double auxError = auxSetPoint - auxSensorSource.get();
            if (Math.abs(auxIntegral) < kIzone[3])
                auxIntegral += auxError * periodMs;

            double auxDerivative = (auxError - prevAuxError) / periodMs;
            output += kP[3] * auxError + kI[3] * auxIntegral + kD[3] * auxDerivative + auxSetPoint * kFF[3];
            prevAuxError = auxError;
        }

        setSpeed.accept(isMotorInverted ? -output : output);
        prevError = error;
    }

    public Command cmdRun(SetMode controlMode, double targetValue, double threshold) {
        return new BuilderCommand(c -> c.endWhen(Math.abs(getError()) < threshold))
            .beforeStarting(() -> set(controlMode, targetValue));
    }

    public Command cmdRun(SetMode controlMode, double targetValue, double auxTargetValue, double threshold) {
        return new BuilderCommand(c -> c.endWhen(Math.abs(getError()) < threshold))
            .beforeStarting(() -> set(controlMode, targetValue, auxTargetValue));
    }
}