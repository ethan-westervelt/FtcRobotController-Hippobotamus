package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class limelightExperimentalFile extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backleft;
    private DcMotor backRight;
    private ElapsedTime timer = new ElapsedTime();
    double kP = 0.04; //tuned down to try to fix drift
    double kI = 0.0;
    double kD = 0.002;

    double integral = 0;
    double lastError = 0;
    double lastTime = timer.seconds();


    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backleft = hardwareMap.get(DcMotor.class, "back_left");
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //double turretAngle = turret.getCurrentPosition();


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(50); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        waitForStart();
        while (opModeIsActive()) {
            limelight.pipelineSwitch(0);

            LLResult result = limelight.getLatestResult();
            LLStatus status = limelight.getStatus();

            if (result.isValid()) {
                double tx = result.getTx();
                double currentTime = timer.seconds();
                double dt = currentTime - lastTime;//result.getTargetingLatency() / 1000.0; //replaced: currentTime - lastTime;

                // PID terms
                double error = tx;
                integral += error * dt;
                double derivative = (error - lastError) / dt;

                double turn = kP * error + kI * integral + kD * derivative;

                // Deadband to prevent jitter
                if (Math.abs(error) < 0.5) {
                    turn = 0;
                    integral = 0;   // reset integral when aligned
                }

                double leftPower = turn;
                double rightPower = turn;

                double max = Math.abs(rightPower);//Math.max(Math.abs(leftPower), Math.abs(rightPower))

                if (max > 1.0) {
                   leftPower /= max;
                   rightPower /= max;
                }

                lastError = error;
                lastTime = currentTime;

                frontRight.setPower(rightPower);
                frontLeft.setPower(leftPower);
                backleft.setPower(leftPower);
                backRight.setPower(rightPower);

                telemetry.addData("Name", "%s",
                        status.getName());
                telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                        status.getTemp(), status.getCpu(),(int)status.getFps());
                telemetry.addData("Pipeline", "Index: %d, Type: %s",
                        status.getPipelineIndex(), status.getPipelineType());
                //telemetry.addData("max", max);
                telemetry.addData("last time", lastTime);
                telemetry.addData("current time", currentTime);
                telemetry.addData("dt", dt);
                telemetry.addData("turn", turn);
                telemetry.addData("result", "is valid");
                telemetry.addData("tx", tx);
                telemetry.addData("ta", result.getTa());
                //telemetry.addData("leftPower", leftPower);
                telemetry.addData("rightPower", rightPower);
                telemetry.update();
            } else if (!result.isValid()) {
                frontRight.setPower(0.1);
                frontLeft.setPower(0.1);
                backleft.setPower(0.1);
                backRight.setPower(0.1);
                telemetry.addData("result", "invalid");
                telemetry.update();
            }


            // Normalize wheel power

/*
                // Apply to motors
                frontLeft.setPower(leftPower);
                backLeft.setPower(-leftPower);
                frontRight.setPower(rightPower);
                backRight.setPower(-rightPower);

                // Save state
                lastError = error;
                lastTime = currentTime;

                telemetry.addData("tx", tx);
                telemetry.addData("turn", turn);
                telemetry.update();

            } else {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }*/


        }
    }
}