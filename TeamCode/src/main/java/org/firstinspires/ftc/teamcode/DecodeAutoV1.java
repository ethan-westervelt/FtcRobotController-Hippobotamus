package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.json.JSONObject;
import org.json.JSONArray;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.Range;


@Autonomous

public class DecodeAutoV1 extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intake;
    private DcMotorEx flywheel1;
    private Servo blocker;
    private Limelight3A limelight;
    private IMU imu;
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

    public void alignToAprilTag() {
        while (opModeIsActive()) {

            // update IMU → Limelight
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.RADIANS));

            // get fresh Limelight data
            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) break;

            double tx = result.getTx();

            // stop condition
            if (Math.abs(tx) < 1.0) break;

            double turnPower = 0.02 * tx;
            turnPower = Range.clip(turnPower, -0.4, 0.4);

            frontLeft.setPower(-turnPower);
            backLeft.setPower(-turnPower);
            frontRight.setPower(turnPower);
            backRight.setPower(turnPower);
        }

/*    double tagX = -1.4827;
    double tagY =  1.4133;

    while (opModeIsActive()) {
        // 1) update Limelight with fresh IMU yaw every loop
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.RADIANS));

        // 2) get fresh pose
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addData("Align", "No valid Limelight pose");
            telemetry.update();
            stopDrive();
            return;
        }

        Pose3D pose = result.getBotpose_MT2();
        double robotX = pose.getPosition().x;
        double robotY = pose.getPosition().y;
        double robotYaw = pose.getOrientation().getYaw(AngleUnit.DEGREES);

        // 3) compute desired yaw and error
        double dx = tagX - robotX;
        double dy = tagY - robotY;
        double desiredYaw = Math.toDegrees(Math.atan2(dy, dx));

        double error = desiredYaw - robotYaw;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        // 4) stop condition
        if (Math.abs(error) <= toleranceDeg) break;

        // 5) control with small static feedforward to overcome stiction
        double turnPower = kP * error;
        if (Math.abs(turnPower) > 0 && Math.abs(turnPower) < 0.08) {
            turnPower = Math.copySign(0.08, turnPower);
        }
        turnPower = Range.clip(turnPower, -0.4, 0.4);

        frontLeft.setPower(-turnPower);
        backLeft.setPower(-turnPower);
        frontRight.setPower(turnPower);
        backRight.setPower(turnPower);

        telemetry.addData("robotX", "%.3f", robotX);
        telemetry.addData("robotY", "%.3f", robotY);
        telemetry.addData("RobotYaw", "%.2f", robotYaw);
        telemetry.addData("DesiredYaw", "%.2f", desiredYaw);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("TurnPower", "%.3f", turnPower);
        telemetry.update();
    }

    stopDrive();
}*/

//uses the limelight's getBotPose method to calulate the error (distance) betweem where your
//robot is and where you want to be.
/*public void driveToPose(double targetX, double targetY, double targetHeading) {
    //tune these
    double KpXY = 0.4;
    double KpTurn = 0.02;

    while (opModeIsActive()) {

        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
        RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.resetYaw();
        IMU.Parameters parameters = new IMU.Parameters(revHubOrientationOnRobot);
        imu.initialize(parameters);

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        Pose3D botPose = llResult.getBotpose_MT2();

        double robotX = botPose.getPosition().x;
        double robotY = botPose.getPosition().y;
        double robotHeading = botPose.getOrientation().getYaw(AngleUnit.DEGREES);

        //double robotHeading = botPose.getPosition().heading;

        //Compute errors
        double errorX = targetX - robotX;
        double errorY = targetY - robotY;
        //double errorHeading = targetHeading - robotHeading;

        //Normalize heading error to [-180, 180]
       // errorHeading = ((errorHeading + 180) % 360) - 180;

        //Convert field errors to robot-centric commands
        double forward = KpXY * errorY;
        double strafe  = KpXY * errorX;
        //double turn    = KpTurn * errorHeading;

        //Mecanum drive mapping
        double fl = forward + strafe;
        double fr = forward - strafe;
        double bl = forward - strafe;
        double br = forward + strafe;

        //Normalize
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);

        // --- Stop condition ---
        if (Math.abs(errorX) < 0.05 &&
            Math.abs(errorY) < 0.05) {

            stopDrive();
            break;
        }
    }
}*/
    }
    double shootX, shootY, shootHeading;
    void saveStartPose(JSONObject ll) {
        JSONArray botpose = ll.optJSONArray("botpose");
        shootX = botpose.optDouble(0, 0);
        shootY = botpose.optDouble(1, 0);
        shootHeading = botpose.optDouble(5, 0);
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
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        //flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        blocker = hardwareMap.get(Servo.class, "blocker");
        PIDController flywheelPID = new PIDController(0.005, 0.0000, 0.0001);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();
        imu = hardwareMap.get(IMU.class, "imu");

        //orients the imu
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.resetYaw();
        IMU.Parameters parameters = new IMU.Parameters(revHubOrientationOnRobot);
        imu.initialize(parameters);

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

        telemetry.addData("Ethan", "says hello");
        telemetry.update();
        FlywheelShoot flywheel = new FlywheelShoot(flywheel1, 0.005, 0.0, 0.0001, 0.00042);
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("Botpose", llResult.getBotpose_MT2());
            telemetry.update();
        }

        //kinda funny that those 200 lines arent the actual auto code, these next ones are.
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //back up to shooting position
        //  driveForwardInches(-24, 0.75);
        // face the tag directly
        alignToAprilTag();  // face the tag directly
        //spin up the wheel
      /*  while (opModeIsActive() && !flywheel.isAtSpeed(2300, 50)) {//waits to proceed until the flywheel is up to speed
            flywheel.setTargetRPM(2550);
        }
        //turn on the intake
        if (opModeIsActive()) {
            intake.setPower(0.5);
        }
        //pause to shoot
        double wait = timer.seconds() + 2.0;
        while (wait > timer.seconds()) {
           idle();
        }
        //back up
        driveForwardInches(-20, 0.75);
        //small strafe for alignment
        encoderDrive(0.75, -2, 2, -2, 2, 5.0);
        //turn 45 degrees left
        encoderDrive(0.75, -12, 12, -12, 12, 5.0);
        //pickup
        driveForwardInches(33, 0.75);
        sleep(1000);
        driveForwardInches(-33, 0.75);
        //turn 45 degrees right
        encoderDrive(0.75, 12, -12, 12, -12, 5.0);
        //returns to shooting position
        driveForwardInches(20, 0.75);
        sleep(1000);
        driveForwardInches(-20, 0.75);
        encoderDrive(0.75, -12, 12, -12, 12, 5.0);
        encoderDrive(0.75, -34, 34, 34, -34, 5.0);
        driveForwardInches(33, 0.75);
        sleep(1000);
        driveForwardInches(-33, 0.75);
        //encoderDrive(0.75, 10, 10, 10, 10, 5.0);
        /*encoderDrive(0.75, 15, -15, -15, 15, 5.0);
        encoderDrive(0.75, 15, -15, -15, 15, 5.0);
        encoderDrive(0.75, 15, -15, -15, 15, 5.0);*/
        while (opModeIsActive()) {
            //&& frontLeft.isBusy() && frontRight.isBusy()
            //&& backLeft.isBusy() && backRight.isBusy()) {
            idle();
        }

        //driveToPose(0, 0, 0);//for the first intake move. need to find the exact locations via botPose
        /*intake.setPower(1);//turns on the intake
        driveForwardInches(20, 0.5);//intake the balls
        driveToPose(shootX, shootY, shootHeading);//returns to shooting spot
        while (opModeIsActive() && !flywheel.isAtSpeed(2550, 50)) {//waits to proceed until the flywheel is up to speed
            flywheel.setTargetRPM(2550);
        }
        //driveToPose(t2x, t2y, t2h);//fot the second intake move. need to find the exact loctions via botPose
        intake.setPower(1);//turns on the intake
        driveForwardInches(20, 0.5);//intake the balls
        driveToPose(shootX, shootY, shootHeading);//returns to shooting spot
        while (opModeIsActive() && !flywheel.isAtSpeed(2550, 50)) {//waits to proceed until the flywheel is up to speed
            flywheel.setTargetRPM(2550);
        }
        //driveToPose(t3x, t3y, t3h);//for the third intake move. need to find the exact locations via botPose
        intake.setPower(1);//turns on the intake
        driveForwardInches(20, 0.5);//intake the balls
        driveToPose(shootX, shootY, shootHeading);//returns to shooting spot
        while (opModeIsActive() && !flywheel.isAtSpeed(2550, 50)) {//waits to proceed until the flywheel is up to speed
            flywheel.setTargetRPM(2550);
        }
        driveToPose(shootX - 0.3, shootY, shootHeading);//moves off the white line
        stopAll();//stops everything*/
    }
}
