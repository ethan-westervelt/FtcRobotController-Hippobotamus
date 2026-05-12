
package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

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
import java.util.List;


@Autonomous

public class simpleLimelight extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private Limelight3A limelight;

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

    void rotate(double power) {

        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    void driveForward(double power) {

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    double tx, td, ts;

    void setAlignmentMotorPower(LLResult result) {
        if (result.isValid()) {

            tx = 0.3*result.getTx() + 0.7*tx;
            td = 0.3 * (Math.sqrt(result.getTa()) - 1.6) + 0.7*td;  // Including the target here.

            // Use raw corners to get the skew value
            List<List<Double>> corners = result.getFiducialResults().get(0).getTargetCorners();

            // This is the left Y range divided the right Y range
            // tl is top left, br is bottom right, etc.
            double tl_y = corners.get(0).get(1);
            double tr_y = corners.get(1).get(1);
            double br_y = corners.get(2).get(1);
            double bl_y = corners.get(3).get(1);

            // Left height
            double leftHeight = tl_y - bl_y;
            double rightHeight = tr_y - br_y;

            // This TS is the ratio of the right and left height of the trapezoid
            // Using an EWMA because it seems to be noisy
            ts = 0.3*100*(leftHeight / rightHeight  - 1) + 0.7*ts;

        } else {

            // If you don't find a value, reduce the values to zero
            tx = 0.9*tx;
            ts = 0.9*ts;
            td = 0.9*td;
        }

        double forward = -0.7 * td;
        double rotate = 0.06 * tx;
        double strafe = 0.06 * ts;

        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        double maxAbs = 0.0;

        if (Math.abs(fl) > maxAbs)
            maxAbs = Math.abs(fl);

        if (Math.abs(fr) > maxAbs)
            maxAbs = Math.abs(fr);

        if (Math.abs(bl) > maxAbs)
            maxAbs = Math.abs(bl);

        if (Math.abs(br) > maxAbs)
            maxAbs = Math.abs(br);

        // Dead band
        if (maxAbs < 0.01) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        } else {
            if (maxAbs < 1)
                maxAbs = 1.0;

            frontLeft.setPower(fl/maxAbs);
            frontRight.setPower(fr/maxAbs);
            backLeft.setPower(bl/maxAbs);
            backRight.setPower(br/maxAbs);
        }

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

    @Override
    public void runOpMode() {

        // Initialize the drive system variables. you can ignore this. its all good and shouldnt need any changes
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");

        //FRONT_RIGHT
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //BACK_LEFT
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //BACK_RIGHT
        backRight = hardwareMap.get(DcMotor.class, "back_right");

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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        telemetry.addData("RPG", "says hello");
        telemetry.update();

        waitForStart();

        ElapsedTime runTime = new ElapsedTime();

        double dt = 0;
        double t1 = runTime.seconds();
        double td = 0;
        double tx = 0;
        double ts = 0;

        //LLResult result = limelight.getLatestResult();

        while (1==1) {
            setAlignmentMotorPower(limelight.getLatestResult());
        }


    }
}

