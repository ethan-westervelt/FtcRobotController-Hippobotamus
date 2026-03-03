//DON'T MESS WITH THIS IT'S MAGIC
package org.firstinspires.ftc.teamcode;


// We need to import external code (code someone else wrote) to make the robot run
//DON'T CHANGE ANY OF THIS, OR ELSE THINGS WON'T WORK!!!

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;


//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import com.qualcomm.robotcore.util.PIDFController;
//Lines that start with "@" are annotations.
//This annotation, @TeleOp, tells the compiler to expect a TeleOp op mode.
//Without it, the driver station won't be able to use the op mode.

@TeleOp
// "Hippo extends LinearOpMode" means Hippo is a subclass of class LinearOpMode.
//   This means that subclass Hippo gets all the functionality of class LinearOpMode.
public class HippoDecode2026_rpg extends LinearOpMode {

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
    public void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // Declarations for the objects that represent the motors/servos:
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    //private DcMotor stand;
    private DcMotorEx flywheel1;
    private Servo blocker;
    private Servo hood;
    private IMU imu;
    private DcMotor intake;
    private ElapsedTime timer = new ElapsedTime();
    private Limelight3A limelight;
    private DcMotor stand;

    // This is an @Override annotation.
    // Since Hippo is a subclass of LinearOpMode, it 'inherits' the public functions
    //   of LinearOpMode, one of which is 'runOpMode'. By default, this is essentially
    //   an empty function that does nothing. Thus, to add in our own code, we need to
    //   "Override" the existing empty function, replacing it with the code that follows.
    // It's worth noting that Override annotations are not technically necessary, but
    //   they can sometimes help the compiler catch errors in the code.
    @Override

    // Here is the most important function of the program, runOpMode. The code in here
    // begins executing as soon as you hit the INIT button
    public void runOpMode() {

        // The following lines of code link the objects we declared above to the
        //   entries in the config file on the robot. The config file links those entries
        //   to the actual electrical wiring that runs the motors.
        // Essentially, this code links software (code) to hardware (motors, servos, etc.)
        // There are also lines that set behaviors for the motors, such as REVERSE or BRAKE.
        // Most lines that set behaviors say something like "set behavior" and the behavior in all caps.


        //NOTE: WHEEL DIRECTIONS ARE AS IF YOU WERE
        //  FRONT_LEFT
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        //frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// Like this one
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //FRONT_RIGHT
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        //frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //BACK_LEFT
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //BACK_RIGHT
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);


        //FLYWHEEL1
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //IMU
        imu = hardwareMap.get(IMU.class, "imu");

        //INTAKE
        intake = hardwareMap.get(DcMotor.class, "intake");
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);

        //STAND
        stand = hardwareMap.get(DcMotor.class, "stand");

        //BLOCKER
        blocker = hardwareMap.get(Servo.class, "blocker");

        //HOOD
        hood = hardwareMap.get(Servo.class, "hood");

        //LIMELIGHT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        // These lines create 2 instances of the PID controller class.
        // A PID controller has three parts: a P term, an I term, and a D term(surprisingly enough).
        // The P term stands for proportional, and effects how quickly the motor responds to changes in output.
        // A high P value will cause your controller to very quickly and strongly respond to any change.
        // Too high of a value can lead to oscillation and surges of power.
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

        // telemetry.addData shoves the data we give it into a buffer
        // telemetry.update takes the contents of that buffer and outputs it on
        // the driver station. Then it deletes the buffer.
        // This can be useful for checking power levels of motors, or the positioning
        // of servos, but doesn't really 'do anything'.

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Version", "2026 V3");
        telemetry.addData("Caleb", "says hi");
        telemetry.addData("Ethan", "says hello");
        telemetry.addData("Tyler", "also says hi");
        telemetry.addData("Asher", "loves Mr DuBois");
        telemetry.addData("And", "vice versa");
        telemetry.addData("We're going", "to States!");
        telemetry.update();

        // Pretty self explanatory. The program pauses execution on the following line,
        // waiting for the play button to be pressed.
        waitForStart();

        double intakePower;
        double targetRPM;

        double kP = 0.11; //tuned down to try to fix drift
        double kI = 0.0;
        double kD = 0.002;

        double integral = 0;
        double lastError = 0;
        double lastTime = timer.seconds();

        // rpg 2026-02-26 kp to 0.01 from 0.007
        FlywheelShoot flywheel = new FlywheelShoot(flywheel1, 0.01, 0.0, 0.0001, 0.00042);

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

            // Take a stand
            if (gamepad2.x | gamepad2.y) {
                if (gamepad2.x)
                    stand.setPower(1);
                if (gamepad2.y)
                    stand.setPower(-1);
            } else {
                stand.setPower(0);
            }

            //intake power
            if (gamepad2.right_stick_y != 0) {
                intakePower = gamepad2.right_stick_y;
            } else if (gamepad2.left_stick_y != 0) {
                intakePower = gamepad2.left_stick_y / 2;
            } else {
                intakePower = 0;
            }

            double x = -gamepad1.right_stick_x;
            double y = gamepad1.right_stick_y;

            double fl = -(x + y);
            double br = (x + y);
            double fr = -(y - x);
            double bl = -(y - x);

            //---------- GAMEPAD 1--------------
            //Mecanum drive code DON'T TOUCH THIS
            /*if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                double x_slow = -gamepad1.left_stick_x;
                double y_slow = gamepad1.left_stick_y;

                fl = -(y + x) / 2;
                br = (y + x) / 2;
                fr = -(y - x) / 2;
                bl = -(y - x) / 2;
            }*/

            //This is the code for the fast spin
            if (gamepad1.left_trigger != 0) {
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
            if (gamepad1.left_bumper) {
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

            /*if (gamepad1.dpad_up) {
                stand.setTargetPosition(100);
                stand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                stand.setPower(1);
            } else if (gamepad1.dpad_down) {
                stand.setTargetPosition(0);
                stand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                stand.setPower(1);
            }*/

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
            intake.setPower(-intakePower);

            if (gamepad2.dpad_up) {
                hood.setPosition(0.75);
            } else if (gamepad2.dpad_down) {
                hood.setPosition(0.2);
            }


            boolean shootShort = gamepad2.left_bumper;
            boolean shootLong = gamepad2.right_bumper;

            targetRPM = 2100;
            if (shootShort) {
                targetRPM = 2100;

            } else if (shootLong) {
                targetRPM = 3000;

            } else {
                targetRPM = 1850;
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

            if (shootShort || shootLong && flywheel.isAtSpeed(3000, 60)) { //flywheel.isAtSpeed(2350, 200)
                blocker.setPosition(0.3);
            } else {
                blocker.setPosition(0);
            }

            if (gamepad1.dpad_down) {
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

                    //double leftPower = turn;
                    double rightPower = turn;

                    double max = Math.abs(rightPower);//Math.max(Math.abs(leftPower), Math.abs(rightPower))

                    if (max > 1.0) {
                        //leftPower /= max;
                        rightPower /= max;
                    }

                    lastError = error;
                    lastTime = currentTime;

                    frontRight.setPower(rightPower);
                    frontLeft.setPower(-rightPower);
                    backLeft.setPower(rightPower);
                    backRight.setPower(-rightPower);

                }

            }
        }
    }
}