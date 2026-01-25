package org.firstinspires.ftc.teamcode22482;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class FlywheelShoot {

    private DcMotorEx flywheel;
    private PIDController pid;
    private double kF;

    public boolean isAtSpeed(double targetRPM, double toleranceRPM) {
        double currentRPM = getCurrentTPS() / 28.0 * 60.0;
        return Math.abs(currentRPM - targetRPM) <= toleranceRPM;
    }

    public FlywheelShoot(DcMotorEx flywheel, double kP, double kI, double kD, double kF) {
        this.flywheel = flywheel;
        this.pid = new PIDController(kP, kI, kD);
        this.kF = kF;
    }

    public void setTargetRPM(double targetRPM) {
        double targetTPS = (targetRPM / 60.0) * 28.0;
        double currentTPS = flywheel.getVelocity();

        double pidOutput = pid.update(targetTPS, currentTPS);
        pidOutput = Range.clip(pidOutput, -1, 1);

        double output = pidOutput + kF * targetTPS;
        flywheel.setPower(output);
    }

    public double getCurrentTPS() {
        return flywheel.getVelocity();
    }

    public double getTargetTPS(double targetRPM) {
        return (targetRPM / 60.0) * 28.0;
    }
}
