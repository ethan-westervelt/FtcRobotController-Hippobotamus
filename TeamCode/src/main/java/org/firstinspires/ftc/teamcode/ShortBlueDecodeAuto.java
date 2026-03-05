
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

import java.util.List;
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
    // private IMU imu;
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

    // Globals used with the limelight alignment.
    // They are globals because I use a low pass filter on the values.
    double tx = 0; // alignment x loc
    double td = 0; // alignment dist (sqrt(ta))
    double ts = 0; // alignment skew
    void setAlignmentMotorPower(LLResult result) {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (result.isValid()) {

            tx = 0.2*result.getTx() + 0.8*tx;
            td = 0.2 * (Math.sqrt(result.getTa()) - 1.6) + 0.8*td;  // Including the target here.

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
            ts = 0.2*100*(leftHeight / rightHeight  - 1) + 0.8*ts;

            telemetry.addData("TS: ", ts);
            telemetry.update();

        } else {


            // If you don't find a value, reduce the values to zero
            tx = 0.9*tx;
            ts = 0.9*ts;
            td = 0.9*td;

            telemetry.addData("No TS: ", ts);
            telemetry.update();
        }

        double forward = -0.5 * td;
        double rotate = 0.03 * tx;
        double strafe = 0.2 * ts;

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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();

        telemetry.addData("RPG", "says hello");
        telemetry.update();

        FlywheelShoot flywheel = new FlywheelShoot(flywheel1, 0.015, 0.0, 0.0001, 0.00042);
        waitForStart();

        ElapsedTime runTime = new ElapsedTime();

        double targetRPM = 2350; //was 2450

        while (runTime.seconds() < 1.2) {
            flywheel.setTargetRPM(targetRPM);
        }

        //double targetTPS = (targetRPM / 60.0) * 28.0;  // convert RPM → ticks/sec

        // 1. Flywheel spin up
        blocker.setPosition(0); // 0 means closed -- cannot fire
        hood.setPosition(0.3);

        double t1 = runTime.seconds();
        double dt = 0;
        //1. back up to shooting position
        flywheel.setTargetRPM(targetRPM);

        //flywheel.setTargetRPM(targetRPM);
        //intake.setPower(0.6);
        driveForwardInches(-38, 0.75);
        // rotateDegrees(10, 0.50);

        //2. spin up the flywheel
        while (dt < 4) {
            flywheel.setTargetRPM(targetRPM);
            setAlignmentMotorPower(limelight.getLatestResult());
            dt = runTime.seconds() - t1;
            //3. open the gate to shoot
            if (dt > 2.5) {
                blocker.setPosition(0.2);
            }
            if (dt > 2.6) {
                intake.setPower(1.0);
            }
        }

        blocker.setPosition(0);
        //flywheel1.setPower(0);
        //3. turn 45 degrees left
        rotateDegrees(-35, 0.50);

        // ----------------------------------------------------------------

        //4. intake 1st spike mark
        driveRightInches(-18, 0.75);
        driveForwardInches(40, 0.6);
        sleep(500);
        driveForwardInches(-40,0.75);
        rotateDegrees(45,0.50);

        //5. shoot
        dt = 0;
        t1 = runTime.seconds();

        flywheel.setTargetRPM(targetRPM);
        while (dt < 5.5) {
            flywheel.setTargetRPM(targetRPM);
            setAlignmentMotorPower(limelight.getLatestResult());
            dt = runTime.seconds() - t1;
            //3. open the gate to shoot
            if (dt > 3.5) {
                blocker.setPosition(0.2);
            }
            if (dt > 3.6) {
                intake.setPower(1.0);
            }
        }
        //6. reset shooting system for next move
        blocker.setPosition(0);

        // 2nd spike
        //7. turn 45 degrees left
        rotateDegrees(-35, 0.50);

        //8. intake 2nd spike mark
        //intake.setPower(1);
        driveRightInches(-46,0.75);
        driveForwardInches(40,0.6);
        sleep(500);

        driveForwardInches(-40,0.75);
        rotateDegrees(60, 0.50);

        dt = 0;
        t1 = runTime.seconds();

        // Shoot second "spike?"
        flywheel.setTargetRPM(targetRPM);
        while (dt < 5.5) {
            flywheel.setTargetRPM(targetRPM);
            setAlignmentMotorPower(limelight.getLatestResult());
            dt = runTime.seconds() - t1;
            //3. open the gate to shoot
            if (dt > 3.5) {
                blocker.setPosition(0.2);
            }
            if (dt > 3.6) {
                intake.setPower(1);
            }
        }

        // GET OFF THE LINE!
        blocker.setPosition(0);
        driveRightInches(6,1);

    }
}
