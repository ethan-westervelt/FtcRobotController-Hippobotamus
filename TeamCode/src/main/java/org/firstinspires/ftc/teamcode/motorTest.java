package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.util.Range;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@TeleOp

public class motorTest extends LinearOpMode {

    private static final Logger log = LoggerFactory.getLogger(motorTest.class);
    private DcMotor motor;

    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Version", "2026 CRI_RPG_1");
        telemetry.addData("Ethan", "says hello");
        telemetry.addData("Tyler", "also says hi");
        telemetry.addData("Asher", "loves Mr DuBois");
        telemetry.addData("And", "vice versa");
        telemetry.addData("We're going", "to Chicago!");
        telemetry.update();

        waitForStart();

        double motorPower = 0.5;

        while (opModeIsActive()) {
            motor.setPower(motorPower);

            if (gamepad1.a) {
                motorPower =  motorPower + 0.1;
                sleep(500);
            } else if (gamepad1.b) {
                motorPower = motorPower - 0.1;
                sleep(500);
            } else if (gamepad1.x) {
                motorPower = 0;
            }

            telemetry.addData("Motor Power", motorPower);
            telemetry.update();
        }
    }
}
