
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
// import com.qualcomm.robotcore.hardware.IMU;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//import org.json.JSONObject;
//import org.json.JSONArray;
//import org.firstinspires.ftc.teamcode.Limelight;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.LLStatus;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous

public class ShortBlueDecodeAuto extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intake;
    private DcMotorEx flywheel1;
    private Servo blocker;
    private Servo hood;
    private Limelight3A limelight;
    // private IMU imu;
    ElapsedTime timer = new ElapsedTime();

    public void encoderDrive(double speed,
                             int flInches, int frInches,
                             int blInches, int brInches,
                             double timeoutS) {

        int newFL = frontLeft.getCurrentPosition()  + (int)(flInches * COUNTS_PER_INCH);
        int newFR = frontRight.getCurrentPosition() + (int)(frInches * COUNTS_PER_INCH);
        int newBL = backLeft.getCurrentPosition()   + (int)(blInches * COUNTS_PER_INCH);
        int newBR = backRight.getCurrentPosition()  + (int)(brInches * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(newFL);
        frontRight.setTargetPosition(newFR);
        backLeft.setTargetPosition(newBL);
        backRight.setTargetPosition(newBR);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));

        //ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() &&
                timer.seconds() < timeoutS &&
                (frontLeft.isBusy() || frontRight.isBusy() ||
                        backLeft.isBusy()  || backRight.isBusy())) {
            // optional telemetry
            telemetry.addData("fl", "%d %d %f", newFL, frontLeft.getCurrentPosition(), Math.abs(speed));
            telemetry.update();
        }

        // Stop all motion
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Return to normal mode
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    public void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void stopAll() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        intake.setPower(0);
        flywheel1.setPower(0);
    }

    void driveForwardInches(double inches, double power) {
        int ticks = (int)(inches * COUNTS_PER_INCH);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy()
                && backLeft.isBusy() && backRight.isBusy()) {
            idle();
        }

        stopDrive();
    }

    void driveRightInches(double inches, double power) {
        int ticks = (int)(inches * COUNTS_PER_INCH);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double coefFL = 1.0;
        double coefFR = 1.0;
        double coefBL = 1.0;
        double coefBR = 1.0;

        frontLeft.setTargetPosition((int)(ticks * coefFL));
        frontRight.setTargetPosition((int)(-ticks * coefFR));
        backLeft.setTargetPosition((int)(-ticks * coefBL));
        backRight.setTargetPosition((int)(ticks * coefBR));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy()
                && backLeft.isBusy() && backRight.isBusy()) {
            idle();
        }

        // Straighten it up?
        driveForwardInches(inches / 9, power);
        //rotateDegrees(-inches/4, power);

        stopDrive();
    }

    void rotate(double power) {

        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    // Positive degrees means clockwise
    void rotateDegrees(double degrees, double power) {

        double inches = degrees / 4.8;
        int ticks = (int)(inches * COUNTS_PER_INCH);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(-ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(-ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy()
                && backLeft.isBusy() && backRight.isBusy()) {
            idle();
        }

        stopDrive();
    }

    // Calculate the COUNTS_PER_INCH for your specific drive train.
// Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
// For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
// For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
// This is gearing DOWN for less speed and more torque.
// For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
//Need to do this, cpmr = 29?
    static final double     COUNTS_PER_MOTOR_REV    = 28;    // rev motors
    static final double     DRIVE_GEAR_REDUCTION    = 20;     // 20:1
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference

    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    //Change these values to adjust speeds
    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.8;
    static final double     LIFT_SPEED              = 0.2;
    static final double     BACK_SPEED              = 0.5;

    double flywheelSpeed;
    double intakeSpeed = 0.2;
    boolean running = false;
    double kP = 0.03;
    double kI = 0.0;
    double kD = 0.002;

    double integral = 0;
    double lastError = 0;
    double lastTime = timer.seconds();

    @Override
    public void runOpMode() {

        // Initialize the drive system variables. you can ignore this. its all good and shouldnt need any changes
        //FRONT_LEFT
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        //frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// Like this one
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //FRONT_RIGHT
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        //frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //BACK_LEFT
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //BACK_RIGHT
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //LIMELIGHT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        //flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        blocker = hardwareMap.get(Servo.class, "blocker");
        hood = hardwareMap.get(Servo.class, "hood");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("RPG", "says hello");
        telemetry.update();

        FlywheelShoot flywheel = new FlywheelShoot(flywheel1, 0.01, 0.0, 0.0001, 0.00042);
        waitForStart();

        ElapsedTime runTime = new ElapsedTime();
        double targetRPM = 2200;

        while (runTime.seconds() < 1.5) {
            flywheel.setTargetRPM(targetRPM);
        }

        hood.setPosition(0.4);
        //double targetTPS = (targetRPM / 60.0) * 28.0;  // convert RPM → ticks/sec

        // 1. Flywheel spin up
        blocker.setPosition(0); // 0 means closed -- cannot fire

        double t1 = runTime.seconds();
        double dt = 0;
        //1. back up to shooting position
        flywheel1.setPower(0.65);
        /*
        LLResult result = limelight.getLatestResult();

        if (result.isValid()) {
            double tx = result.getTx();
            double currentTime = timer .seconds();
            double t = currentTime - lastTime;

            // PID terms
            double error = tx;
            integral += error * t;
            double derivative = (error - lastError) / t;

            double turn = kP * error + kI * integral + kD * derivative;

            // Deadband to prevent jitter
            if (Math.abs(error) < 0.25) {
                turn = 0;
                integral = 0;   // reset integral when aligned
            }

            // Normalize wheel power
            double leftPower = -turn;
            double rightPower = turn;

            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

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
        }
        */



        //flywheel.setTargetRPM(targetRPM);
        intake.setPower(0.2);
        driveForwardInches(-38, 0.8);
        rotateDegrees(-10, 0.50);

        LLResult result = limelight.getLatestResult();

        //2. spin up the flywheel
        while (dt < 5.5) {
            flywheel.setTargetRPM(targetRPM);
            dt = runTime.seconds() - t1;

            result = limelight.getLatestResult();
            if (result.isValid()) {
                double tx = result.getTx();
                rotate(tx*0.3);
            }

            //3. open the gate to shoot
            if (dt > 2.5) {
                blocker.setPosition(0.7);
            }
            if (dt > 2.5) {
                intake.setPower(0.6);
            }
        }

        //blocker.setPosition(0);
        flywheel1.setPower(0);
        //3. turn 45 degrees left
        rotateDegrees(-44, 0.50);

        blocker.setPosition(0);

        // ----------------------------------------------------------------
        //4. intake 1st spike mark
        intake.setPower(1);
        sleep(500);
        driveRightInches(-14, 0.8);
        driveForwardInches(30, 0.6);
        sleep(500);
        flywheel1.setPower(0.65);
        driveForwardInches(-14,0.8);
        driveRightInches(16, 0.8);
        rotateDegrees(50,0.8);
        driveForwardInches(-8,0.8);

        //5. shoot
        dt = 0;
        t1 = runTime.seconds();

        flywheel.setTargetRPM(targetRPM);
        while (dt < 4.5) {
            flywheel.setTargetRPM(targetRPM);
            dt = runTime.seconds() - t1;

            result = limelight.getLatestResult();
            if (result.isValid()) {
                double tx = result.getTx();
                rotate(tx*0.3);
            }

            //3. open the gate to shoot
            if (dt > 1.5) {
                blocker.setPosition(0.7);
            }
            if (dt > 1.6) {
                intake.setPower(0.65);
            }
        }
        //6. reset for next move
        flywheel1.setPower(0);
        blocker.setPosition(0);

        // --------------------------------------------------------------------
        // 2nd spike
        //7. turn 45 degrees left
        rotateDegrees(-45, 0.8);

        //8. intake 2nd spike mark
        intake.setPower(1);
        //driveForwardInches(-6, 0.8);
        driveRightInches(-36,0.8);
        driveForwardInches(30,0.6);
        sleep(500);

        driveForwardInches(-8,0.8);
        rotateDegrees(45, 0.8);
        flywheel1.setPower(0.65);
        driveRightInches(40,0.8);
        rotateDegrees(5,0.8);
        //driveForwardInches(7,0.83);


        dt = 0;
        t1 = runTime.seconds();
        /*
        if (result.isValid()) {
            double tx = result.getTx();
            double currentTime = timer .seconds();
            double t = currentTime - lastTime;

            // PID terms
            double error = tx;
            integral += error * t;
            double derivative = (error - lastError) / t;

            double turn = kP * error + kI * integral + kD * derivative;

            // Deadband to prevent jitter
            if (Math.abs(error) < 0.25) {
                turn = 0;
                integral = 0;   // reset integral when aligned
            }

            // Normalize wheel power
            double leftPower = -turn;
            double rightPower = turn;

            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

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
        }

         */

        // Shoot second "spike?"
        flywheel.setTargetRPM(targetRPM);
        while (dt < 4.5) {
            flywheel.setTargetRPM(targetRPM);
            dt = runTime.seconds() - t1;

            result = limelight.getLatestResult();
            if (result.isValid()) {
                double tx = result.getTx();
                rotate(tx*0.5);
            }

            //3. open the gate to shoot
            if (dt > 1.5) {
                blocker.setPosition(0.7);
            }
            if (dt > 1.6) {
                intake.setPower(0.65);
            }
        }

        // GET OFF THE LINE!
        driveRightInches(-24,1);
        blocker.setPosition(0);

    }
}
