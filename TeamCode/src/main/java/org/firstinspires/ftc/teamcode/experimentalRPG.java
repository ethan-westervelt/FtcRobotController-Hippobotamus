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
public class experimentalRPG extends LinearOpMode {

    //import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
    public void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    double tx = 0;

    void setAlignmentRotatePower(LLResult result, double targetOffset) {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (result.isValid()) {
            tx = 0.3 * result.getTx() + 0.7 * tx;
        } else {
            tx = 0.9 * tx;
        }

        double rotate = 0.03 * (tx - targetOffset);

        // Dead band
        if (rotate < 0.01) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        } else {
            frontLeft.setPower(rotate);
            frontRight.setPower(-rotate);
            backLeft.setPower(rotate);
            backRight.setPower(-rotate);
        }

    }

    double calcTurretAlignmentPower(LLResult result) {
        if (result.isValid()) {
            tx = 0.3 * result.getTx() + 0.7 * tx;
        } else {
            tx = 0.9 * tx;
        }

        double turretPower = 0;
        if (Math.abs(tx) > 0.1) {
            turretPower = -tx * 0.03;
        }
        return(turretPower);
    }

    // Declarations for the objects that represent the motors/servos:
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor turret;
    private DcMotorEx flywheel1;
    private Servo blocker;
    private Servo hood;
    private DcMotor intake;
    private ElapsedTime timer = new ElapsedTime();
    private Limelight3A limelight;


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

        //  FRONT_LEFT
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //FRONT_RIGHT
        frontRight = hardwareMap.get(DcMotor.class, "front_right");

        //BACK_LEFT
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //BACK_RIGHT
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        //FLYWHEEL1
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //INTAKE
        intake = hardwareMap.get(DcMotor.class, "intake");

        //BLOCKER
        blocker = hardwareMap.get(Servo.class, "blocker");

        //HOOD
        hood = hardwareMap.get(Servo.class, "hood");

        // TURRET
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // You need this many ticks of the motor to get one degree of turret rotation
        double turretEncPerDeg = -5.63;

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

        flywheel1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // telemetry.addData shoves the data we give it into a buffer
        // telemetry.update takes the contents of that buffer and outputs it on
        // the driver station. Then it deletes the buffer.
        // This can be useful for checking power levels of motors, or the positioning
        // of servos, but doesn't really 'do anything'.

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Version", "2026 CRI_RPG_1");
        telemetry.addData("Ethan", "says hello");
        telemetry.addData("Tyler", "also says hi");
        telemetry.addData("Asher", "loves Mr DuBois");
        telemetry.addData("And", "vice versa");
        telemetry.addData("We're going", "to Chicago!");
        telemetry.update();

        // Pretty self explanatory. The program pauses execution on the following line,
        // waiting for the play button to be pressed.
        waitForStart();

        double intakePower;
        double targetRPM;

        double kP = 0.06; //tuned down to try to fix drift
        double kI = 0.0;
        double kD = 0.002;

        double integral = 0;
        double lastError = 0;
        double lastTime = timer.seconds();

        FlywheelShoot flywheel = new FlywheelShoot(flywheel1, 0.016, 0.0, 0.0001, 0.00042);

        double fl = 0;
        double fr = 0;
        double br = 0;
        double bl = 0;
        double rotate = 0;
        double turretPower = 0;

        while (opModeIsActive()) {

            // -------------------------------
            // ----------GAMEPAD 1 ------------
            // -------------------------------

            // Forward (y) and strafe (x)
            double x = -gamepad1.right_stick_x;
            double y = gamepad1.right_stick_y;

            // Trigger is fast spin, bumpers are slow spin
            if (gamepad1.left_trigger > 0) {
                rotate = -1;
            } else if (gamepad1.right_trigger > 0) {
                rotate = 1;
            } else if (gamepad1.left_bumper) {
                rotate = -0.3;
            } else if (gamepad1.right_bumper) {
                rotate = 0.3;
            } else {
                rotate = 0;
            }

            // Power to the wheels are the sum of X, Y, and rotate
            fl = y + x + rotate;
            bl = y - x + rotate;
            fr = y - x - rotate;
            br = y + x - rotate;

            // Normalize to the maximum power to a wheel
            double maxPower = 1.0;
            if (Math.abs(fl) > maxPower)
                maxPower = Math.abs(fl);
            if (Math.abs(bl) > maxPower)
                maxPower = Math.abs(bl);
            if (Math.abs(fr) > maxPower)
                maxPower = Math.abs(fr);
            if (Math.abs(br) > maxPower)
                maxPower = Math.abs(br);

            if (maxPower > 1)
                maxPower = 1;

            fl = fl / maxPower;
            bl = bl / maxPower;
            fr = fr / maxPower;
            br = br / maxPower;

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // -------------------------------
            // ----------GAMEPAD 2------------
            // -------------------------------

            // turret power!
            // Either use manual bumpers or the left trigger for auto align.
            // Positive power means counter clockwise.
            turretPower = 0;
            double turretPos = turret.getCurrentPosition() / turretEncPerDeg;
            if (gamepad2.right_bumper) {
                turretPower = -0.6;
            }
            if (gamepad2.left_bumper) {
                turretPower = 0.6;
            }
            // Turret auto centering
            if (gamepad2.left_trigger_pressed) {
                turretPower = calcTurretAlignmentPower(limelight.getLatestResult());
            }

            // If you're too far counter clockwise you can only have negative power
            if (turretPos < -200) {
                turretPower = Math.min(0,turretPower);
            }
            // Likewise, if you're too far clockwise, you can only have positive power
            if (turretPos > 200) {
                turretPower = Math.max(0,turretPower);
            }
            turret.setPower(turretPower);

            //intake power
            if (gamepad2.right_stick_y != 0) {
                intakePower = gamepad2.right_stick_y;
            } else if (gamepad2.left_stick_y != 0) {
                intakePower = gamepad2.left_stick_y / 2;
            } else {
                intakePower = 0;
            }

            if (gamepad2.y) {
                blocker.setPosition(0);
            } else if (gamepad2.b) {
                blocker.setPosition(0.2);
            }


            intake.setPower(-intakePower);

            if (gamepad2.dpad_up) {
                hood.setPosition(0.75);
            } else if (gamepad2.dpad_down) {
                hood.setPosition(0.2);
            }


            // Auto centering.  If you are shooting short and a big far
            // then adjust the distance multiplier.
            double distanceMult = 1.0;
            if (gamepad1.dpad_down && gamepad2.right_bumper) {
                LLResult result = limelight.getLatestResult();
                LLStatus status = limelight.getStatus();

                if (result.isValid()) {
                    double tx = result.getTx();
                    double ta = result.getTa();
                    setAlignmentRotatePower(limelight.getLatestResult(), -3.0);
                    distanceMult = 20.4 / (ta + 18);
                }
            } else if (gamepad1.dpad_down) {
                LLResult result = limelight.getLatestResult();
                LLStatus status = limelight.getStatus();

                if (result.isValid()) {
                    double tx = result.getTx();
                    double ta = result.getTa();
                    setAlignmentRotatePower(limelight.getLatestResult(), 0);
                    distanceMult = 20.4 / (ta + 18);
                }
            }


            boolean shootShort = gamepad2.left_bumper;
            boolean shootLong = gamepad2.right_bumper;

            targetRPM = 2100;
            if (shootShort) {
                targetRPM = 2100 * distanceMult;

            } else if (shootLong) {
                targetRPM = 3100;

            } else {
                targetRPM = 1850;
            }

            if (gamepad2.x) {
                flywheel1.setPower(-1);
            }

            flywheel.setTargetRPM(targetRPM);

            // Read velocity in ticks/sec

            double currentTPS = flywheel1.getVelocity();
            double targetTPS = (targetRPM / 60.0) * 28.0;

            if ((targetRPM > 0) && (Math.abs(currentTPS - targetTPS) < 10)) {
                gamepad2.rumble(100);
            }

            if (shootShort || shootLong && flywheel.isAtSpeed(3100, 60)) { //flywheel.isAtSpeed(2350, 200)
                blocker.setPosition(0.3);
            } else {
                blocker.setPosition(0);
            }

        }
    }
}