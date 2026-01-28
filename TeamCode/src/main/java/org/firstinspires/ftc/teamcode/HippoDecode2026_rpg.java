//DON'T MESS WITH THIS IT'S MAGIC
package org.firstinspires.ftc.teamcode;

// We need to import external code (code someone else wrote) to make the robot run
//DON'T CHANGE ANY OF THIS, OR ELSE THINGS WON'T WORK!!!
import android.util.Size;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.Timer;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.FlywheelShoot;
import org.json.JSONObject;
import org.json.JSONArray;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import com.qualcomm.robotcore.util.PIDFController;
//Lines that start with "@" are annotations.
//This annotation, @TeleOp, tells the compiler to expect a TeleOp op mode.
//Without it, the driver station won't be able to use the op mode.

@TeleOp
// "Hippo extends LinearOpMode" means Hippo is a subclass of class LinearOpMode.
//   This means that subclass Hippo gets all the functionality of class LinearOpMode.
public class HippoDecode2026_rpg extends LinearOpMode {

    public void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    void rotateDegrees(double degrees, double power) {

        double     COUNTS_PER_MOTOR_REV    = 28;    // rev motors
        double     DRIVE_GEAR_REDUCTION    = 20;     // 20:1
        double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference

        double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
//Change these values to adjust speeds

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

    // Declarations for the objects that represent the motors/servos:
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotorEx flywheel1;

    private DcMotor leg1;
    private DcMotor leg2;

    private Servo blocker;
    private IMU imu;
    private DcMotor intake;
    private DcMotor conveyorBelt;

    private ElapsedTime timer = new ElapsedTime();
    private VoltageSensor voltageSensor;
    private AprilTagAim autoAim;
    private Limelight3A limelight;

    // This is an @Override annotation.
    // Since Hippo is a subclass of LinearOpMode, it 'inherits' the public functions
    //   of LinearOpMode, one of which is 'runOpMode'. By default, this is essentially
    //   an empty function that does nothing. Thus, to add in our own code, we need to
    //   "Override" the existing empty function, replacing it with the code that follows.
    // It's worth noting that Override annotations are not technically necessary, but
    //   they can sometimes help the compiler catch errors in the code.

    // Here is the most important function of the program, runOpMode. The code in here
    // begins executing as soon as you hit the INIT button
    public void runOpMode() {

        // The following lines of code link the objects we declared above to the
        //   entries in the config file on the robot. The config file links those entries
        //   to the actual electrical wiring that runs the motors.
        // Essentially, this code links software (code) to hardware (motors, servos, etc.)
        // There are also lines that set behaviors for the motors, such as REVERSE or BRAKE.
        // Most lines that set behaviors say something like "set behavior" and the behavior in all caps.
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        //frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// Like this one
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        //frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        leg1 = hardwareMap.get(DcMotor.class, "leg1");
        leg1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leg1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leg1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leg2 = hardwareMap.get(DcMotor.class, "leg2");
        leg2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leg2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leg2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leg2.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        blocker =  hardwareMap.get(Servo.class, "blocker");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!


        // These lines create 2 instances of the PID controller class.
        // A PID controller has three parts: a P term, an I term, and a D term(suprisingly enough).
        // The P term stands for proportional, and effects how quickly the motor responds to changes in output.
        // A high P value will cause your controller to very quickly and strongly respond to any change.
        // Too high of a value can lead to ocillation and surges of power.
        // A low P value means that your controller will take a very long time to reach its target and will recover slowly.
        // The D term is derivative, and helps dampens small changes.

        //initialize and calibrate the imu for navigation
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.resetYaw();
        IMU.Parameters parameters = new IMU.Parameters(revHubOrientationOnRobot);
        imu.initialize(parameters);

        flywheel1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //adds the apriltag processor and webcam
       /* AprilTagProcessor aprilTag = new AprilTagProcessor.Builder().build();
        VisionPortal visionPortal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .addProcessor(aprilTag)
        .build();*/

        // telemetry.addData shoves the data we give it into a buffer
        // telemetry.update takes the contents of that buffer and outputs it on
        // the driver station. Then it deletes the buffer.
        // This can be useful for checking power levels of motors, or the positioning
        // of servos, but doesn't really 'do anything'.

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Version", "3.3.0.1");
        telemetry.addData("Caleb", "says hi");
        telemetry.addData("Ethan","says hello");
        telemetry.addData("Tyler","also says hi");
        telemetry.addData("Asher", "loves Evelyn");
        telemetry.addData("And","vice versa");
        telemetry.update();

        // Pretty self explanatory. The program pauses execution on the following line,
        // waiting for the play button to be pressed.
        waitForStart();

        // Declarations of variables representing the power levels of each of the four
        //   drive motors. These are all on a scale from 0 to 1 (any values greater than 1
        //   or less than 0 will basically act the same as just a 1 or a 0)
        //double fl; //front-left power
        //double fr; //front-right power
        //double bl; //back-left power
        //double br; //back-right power
        double flywheelSpeed = 0; //flywheel power, 10 is an estimated target voltage
        double spin_modifier = 0;
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
        double roll  = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
        double intakePower;
        double conveyorBeltPower;
        double targetRPM;
        int legTarget;
        double legPower = 1;

        double kP = 0.03;
        double kI = 0.0;
        double kD = 0.002;

        double integral = 0;
        double lastError = 0;
        double lastTime = timer.seconds();

        FlywheelShoot flywheel = new FlywheelShoot(flywheel1, 0.005, 0.0, 0.0001, 0.00042);

        // "power_multiplier" is a general value that allows us to control the global
        //   power level of the drive motors. Although currently useless, if we ever
        //   wanted a control to slow down the speed of the robot, we would need to use
        //   this variable
        double power_multiplier = -1;
        // This is the main loop of the program, the stuff that runs continuously
        //   while the op mode is running. "opModeIsActive()" is a function that returns
        //   true so long as the op mode is active/running, so as soon as we hit stop
        //   the loop will terminate.
        while (opModeIsActive()) {

            // Some more telemetry data. Note that once telemetry is updated, the previous telemetry data will be erased.
            //telemetry.addData("Status", "Running");
            //telemetry.addData("Heading", heading);
            //telemetry.addData("Pitch", pitch);
            //telemetry.addData("Roll", roll);

           /* YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            limelight.updateRobotOrientation(orientation.getYaw());
            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                Pose3D botPose = llResult.getBotpose_MT2();
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
                telemetry.addData("Botpose", llResult.getBotpose_MT2());
                telemetry.update();
            }*/

            //intake power
            if (gamepad2.right_stick_y != 0) {
                intakePower = gamepad2.right_stick_y;
            } else if (gamepad2.left_stick_y != 0) {
                intakePower = gamepad2.left_stick_y / 2;
            } else {
                intakePower = 0;
            }

            //declare the imu in the control hub
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
            roll  = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);

            double x = -gamepad1.right_stick_x;
            double y = gamepad1.right_stick_y;

            double fl = -(x + y);
            double br = (x + y);
            double fr = -(y - x);
            double bl = -(y - x);
            //---------- GAMEPAD 1--------------
            //Mecanum drive code DONT TOUCH THIS
            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                double x_slow = -gamepad1.left_stick_x;
                double y_slow = gamepad1.left_stick_y;

                fl = -(y + x) / 2;
                br = (y + x) / 2;
                fr = -(y - x) / 2;
                bl = -(y - x) / 2;
            }

            //This is the code for the fast spin
            if (gamepad1.left_trigger != 0){
                fr = 1;
                fl = -1;
                br = -1;
                bl = -1;
            } else if (gamepad1.right_trigger != 0) {
                fl = 1;
                fr = -1;
                bl = 1;
                br = 1;
            }

            //Slow spin
            if (gamepad1.left_bumper){
                fr = 0.3;
                fl = -0.3;
                br = -0.3;
                bl = -0.3;
            } else if (gamepad1.right_bumper) {
                fl = 0.3;
                fr = -0.3;
                bl = 0.3;
                br = 0.3;
            }

            //under construction...


            //----------GAMEPAD 2------------
            if (gamepad2.y) {
                blocker.setPosition(0);
            } else if (gamepad2.b) {
                blocker.setPosition(0.2);
            }

            frontLeft.setPower(-fl);
            frontRight.setPower(-fr);
            backLeft.setPower(bl);
            backRight.setPower(-br);
            intake.setPower(intakePower);
            targetRPM = 2350;

            boolean shootShort = gamepad2.left_bumper;
            boolean shootLong = gamepad2.right_bumper;

            targetRPM = 0;
            if (shootShort) {
                //targetRPM = 2350;

                //LIMELIGHT AUTO ALIGNER
                LLResult result = limelight.getLatestResult();

                if (result.isValid()) {
                    double tx = result.getTx();
                    double currentTime = timer .seconds();
                    double dt = currentTime - lastTime;

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
            } else if (shootLong) {
                targetRPM = 3400;

                //LIMELIGHT AUTO ALIGNER
                LLResult result = limelight.getLatestResult();

                if (result.isValid()) {
                    double tx = result.getTx();
                    double currentTime = timer .seconds();
                    double dt = currentTime - lastTime;

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
            }   else {
                targetRPM = 0;
            }

            if (gamepad2.x) {
                flywheel1.setPower(-1);
            }

            flywheel.setTargetRPM(targetRPM);

            // Read velocity in ticks/sec
            double currentTPS = flywheel1.getVelocity();   // ticks per second
            double targetTPS = (targetRPM / 60.0) * 28.0;  // convert RPM → ticks/sec
            telemetry.addData("currentTPS", currentTPS);
            telemetry.addData("targetTPS", targetTPS);
            telemetry.update();

            if ((targetRPM > 0) && (Math.abs(currentTPS - targetTPS) < 10)) {
                gamepad2.rumble(100);
            }

            if (shootShort && flywheel.isAtSpeed(2350, 40)) {
                blocker.setPosition(1);
                if (!gamepad2.left_bumper) {
                    blocker.setPosition(0);
                }
            }

            if (shootLong && flywheel.isAtSpeed(3400, 60)) {
                blocker.setPosition(1);
                if (!gamepad2.right_bumper) {
                    blocker.setPosition(0);
                }
            }

            if (targetRPM == 0)  {
                blocker.setPosition(0);
                //flywheel1.setPower(0);
                //flywheelPID.reset();
            }

        }
    }

}