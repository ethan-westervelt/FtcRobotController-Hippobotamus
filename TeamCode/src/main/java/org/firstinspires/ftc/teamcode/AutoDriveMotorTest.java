package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;




@Autonomous

public class AutoDriveMotorTest extends LinearOpMode {

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    ElapsedTime runTime = new ElapsedTime();
    double dt = 0;
    double totalTime = 0;
    double t1 = runTime.seconds();
    double startTime;


    @Override
    public void runOpMode() {

        // Initialize the drive system variables. you can ignore this. its all good and shouldnt need any changes
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidf = backLeft.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addData("RPG", "says hello");
        telemetry.addData("P: ", pidf.p);
        telemetry.addData("I: ", pidf.i);
        telemetry.addData("D: ", pidf.d);
        telemetry.addData("F: ", pidf.f);
        telemetry.update();

        waitForStart();
        startTime = runTime.seconds();

        double powerTarget = 0.2;
        double velocityTarget = 600;

        /*
        frontLeft.setPower(powerTarget);
        frontRight.setPower(powerTarget);
        backLeft.setPower(powerTarget);
        backRight.setPower(powerTarget);
         */

        frontLeft.setVelocity(velocityTarget);
        frontRight.setVelocity(velocityTarget);
        backLeft.setVelocity(velocityTarget);
        backRight.setVelocity(velocityTarget);

        while (1==1) {

            if (dt < 0.5) {
                dt = runTime.seconds() - t1;
            } else {
                telemetry.addData("Velocity Target: ", velocityTarget);
                telemetry.addData("FR: ", frontRight.getVelocity());
                telemetry.addData("FL: ", frontLeft.getVelocity());
                telemetry.addData("BR: ", backRight.getVelocity());
                telemetry.addData("BL: ", backLeft.getVelocity());
                telemetry.update();

                frontLeft.setVelocity(velocityTarget);
                frontRight.setVelocity(velocityTarget);
                backLeft.setVelocity(velocityTarget);
                backRight.setVelocity(velocityTarget);

                /*
                frontLeft.setPower(powerTarget);
                frontRight.setPower(powerTarget);
                backLeft.setPower(powerTarget);
                backRight.setPower(powerTarget);
                 */

                dt = 0;
                t1 = runTime.seconds();

                if ((t1 - startTime) > 5) {
                    powerTarget = 0.5;
                    velocityTarget = 900;
                }

                if ((t1 - startTime) > 10) {
                    powerTarget = 0.8;
                    velocityTarget = -900;
                }

                if ((t1 - startTime) > 15) {
                    powerTarget = 1.0;
                    velocityTarget = 1800;
                }

                if ((t1 - startTime) > 20) {
                    powerTarget = 0.1;
                    velocityTarget = 200;
                }

                if ((t1 - startTime) > 25) {
                    powerTarget = 0.2;
                    velocityTarget = 600;
                }

            }
        }

    }
}
