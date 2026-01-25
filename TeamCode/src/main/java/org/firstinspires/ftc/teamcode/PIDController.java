package org.firstinspires.ftc.teamcode;

public class PIDController {
    private double kP, kI, kD;
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        lastTime = System.nanoTime() / 1e9;
    }

    public double update(double target, double current) {
        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        if (dt <= 0) return 0;

        double error = target - current;
        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        return (kP * error) + (kI * integralSum) + (kD * derivative);
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastTime = System.nanoTime() / 1e9;
    }
}