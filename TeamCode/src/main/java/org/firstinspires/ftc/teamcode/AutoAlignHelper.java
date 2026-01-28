package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoAlignHelper {

    private final Limelight3A limelight;
    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final Telemetry telemetry;

    private final ElapsedTime timer = new ElapsedTime();

    // PID constants
    private double kP = 0.03;
    private double kI = 0.0;
    private double kD = 0.002;

    // PID state
    private double integral = 0;
    private double lastError = 0;
    private double lastTime = 0;

    public AutoAlignHelper(Limelight3A limelight,
                           DcMotor frontLeft, DcMotor frontRight,
                           DcMotor backLeft, DcMotor backRight,
                           Telemetry telemetry) {

        this.limelight = limelight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.telemetry = telemetry;

        lastTime = timer.seconds();
    }

    /** Call this repeatedly inside your OpMode loop */
    public void update() {
        LLResult result = limelight.getLatestResult();

        if (!result.isValid()) {
            stopMotors();
            return;
        }

        double tx = result.getTx();
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;

        // PID calculations
        double error = tx;
        integral += error * dt;
        double derivative = (error - lastError) / dt;

        double turn = kP * error + kI * integral + kD * derivative;

        // Deadband
        if (Math.abs(error) < 0.5) {
            turn = 0;
            integral = 0;
        }

        // Motor power
        double leftPower = -turn;
        double rightPower = turn;

        // Normalize
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        // Apply power
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);

        // Save state
        lastError = error;
        lastTime = currentTime;

        telemetry.addData("tx", tx);
        telemetry.addData("turn", turn);
        telemetry.update();
    }

    public void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}