package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

@TeleOp

public class SOLVERSLIB_DRIVETEST extends LinearOpMode {

    private Motor frontLeft = new Motor(hardwareMap, "frontLeft");
    private Motor frontRight = new Motor(hardwareMap, "frontRight");
    private Motor backLeft = new Motor(hardwareMap, "backLeft");
    private Motor backRight = new Motor(hardwareMap, "backRight");

    DcMotor fl = frontLeft.motor;
    DcMotor fr = frontRight.motor;
    DcMotor bl = backLeft.motor;
    DcMotor br = backRight.motor;
    private MecanumDrive mecanum = new MecanumDrive(frontLeft, frontRight,
            backLeft, backRight);
    private GamepadEx driverOp = new GamepadEx(gamepad1);


    public void runOpMode() {


      while (opModeIsActive()) {

          //hypothetically does robot-centric mecanum drive
          mecanum.driveRobotCentric(
                  driverOp.getLeftX(),
                  driverOp.getLeftY(),
                  driverOp.getRightY()
          );

          telemetry.addData("LEFT X",driverOp.getLeftX());
          telemetry.addData("LEFT Y",driverOp.getLeftY());
          telemetry.addData("RIGHT Y",driverOp.getRightY());
          telemetry.update();
      }


    }
}

