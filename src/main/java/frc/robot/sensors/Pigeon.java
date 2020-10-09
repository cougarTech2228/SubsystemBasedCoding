package frc.robot.sensors;

import com.ctre.phoenix.sensors.PigeonIMU;

public class Pigeon {
    private PigeonIMU pidgey;
    private double[] data;
    public Pigeon(int port) {
        pidgey = new PigeonIMU(port);
        data = new double[3];
    }

    public double getYaw() {
        update();
        return data[0];
    }
    public double getPitch() {
        update();
        return data[1];
    }
    public double getRoll() {
        update();
        return data[2];
    }
    public double getContinuousYaw() {
        update();
        double num = data[0] % 360; //-360 - 360
        if(num < -180) num += 360;
        if(num > 180) num -= 360;

        return num;
    }
    private void update() {
        pidgey.getYawPitchRoll(data);
    }
}