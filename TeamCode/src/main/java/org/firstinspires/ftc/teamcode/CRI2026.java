//DON'T MESS WITH THIS IT'S MAGIC
package org.firstinspires.ftc.teamcode;

// We need to import external code (code someone else wrote) to make the robot run
//DON'T CHANGE ANY OF THIS, OR ELSE THINGS WON'T WORK!!!

import java.lang.Math;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

//Lines that start with "@" are annotations.
//This annotation, @TeleOp, tells the compiler to expect a TeleOp op mode.
//Without it, the driver station won't be able to use the op mode.

@TeleOp
// "Hippo extends LinearOpMode" means Hippo is a subclass of class LinearOpMode.
//   This means that subclass Hippo gets all the functionality of class LinearOpMode.
public class CRI2026 extends LinearOpMode {

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


    // Declarations for the objects that represent the motors/servos:
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo hood;
    private Limelight3A limelight;

    private DcMotorEx flywheel1;

    private Servo blocker;

    private DcMotor intake;
    private DcMotor roller;


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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        // The following lines of code link the objects we declared above to the
        //   entries in the config file on the robot. The config file links those entries
        //   to the actual electrical wiring that runs the motors.
        // Essentially, this code links software (code) to hardware (motors, servos, etc.)
        // There are also lines that set behaviors for the motors, such as REVERSE or BRAKE.
        // Most lines that set behaviors say something like "set behavior" and the behavior in all caps.
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");

        frontRight = hardwareMap.get(DcMotor.class, "front_right");

        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        backRight = hardwareMap.get(DcMotor.class, "back_right");

        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);

        roller = hardwareMap.get(DcMotor.class, "roller");
        roller.setDirection(DcMotorSimple.Direction.REVERSE);

        blocker =  hardwareMap.get(Servo.class, "blocker");

        hood = hardwareMap.get(Servo.class, "hood");

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
        telemetry.addData("Version", "3.3.0.1");
        telemetry.addData("Caleb", "says hi");
        telemetry.addData("Ethan","says hello");
        telemetry.addData("Tyler","also says hi");
        telemetry.addData("Asher", "loves Evelyn");
        telemetry.addData("And","vice versa");
        telemetry.update();

        // Pretty self-explanatory. The program pauses execution on the following line,
        // waiting for the play button to be pressed.
        waitForStart();

        // Declarations of variables representing the power levels of each of the four 
        //   drive motors. These are all on a scale from 0 to 1 (any values greater than 1
        //   or less than 0 will basically act the same as just a 1 or a 0)
        double rollerPower;
        double intakePower;
        double targetRPM;


        FlywheelShoot flywheel = new FlywheelShoot(flywheel1, 0.005, 0.0, 0.0001, 0.00042);

        // "power_multiplier" is a general value that allows us to control the global
        //   power level of the drive motors. Although currently useless, if we ever
        //   wanted a control to slow down the speed of the robot, we would need to use 
        //   this variable
        double power_multiplier = 1;
        // This is the main loop of the program, the stuff that runs continuously 
        //   while the op mode is running. "opModeIsActive()" is a function that returns
        //   true so long as the op mode is active/running, so as soon as we hit stop 
        //   the loop will terminate.
        while (opModeIsActive()) {

                LLResult result = limelight.getLatestResult();

                if (result.isValid()) {
                    // Access general information
                    Pose3D botpose = result.getBotpose();
                    double power;
                    double captureLatency = result.getCaptureLatency();
                    double targetingLatency = result.getTargetingLatency();
                    double parseLatency = result.getParseLatency();
                    double tx = result.getTx();
                    double ty = result.getTy();
                    double ta = result.getTa();

                    //just tells you what tag we're looking at
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }

                    if (Math.abs(tx) < 0.5) {
                        power = 0;
                    }

                    double kP = 0.04; //power constant
                    power = kP * tx;

                    frontLeft.setPower(-power);
                    frontRight.setPower(power);
                    backLeft.setPower(-power);
                    backRight.setPower(power);

                    telemetry.addData("tx", tx);
                    telemetry.addData("turn power", power);
                    telemetry.update();

                }

            //----------GAMEPAD 1------------

            //roller power (normal)
            /*
            if (gamepad2.right_stick_y != 0) {
                rollerPower = gamepad2.right_stick_y;
            } else if (gamepad2.left_stick_y != 0) {
                rollerPower = gamepad2.left_stick_y / 2;
            } else {
                rollerPower = 0;
            }*/


            //intake and roller so asher can test
            //intake:

            rollerPower = 0;
            intakePower = 0;
            if (gamepad2.right_stick_y != 0) {
                intakePower = -gamepad2.right_stick_y;
            }
            if (gamepad2.left_stick_y != 0) {
                rollerPower = gamepad2.left_stick_y;
            }

            double x = -gamepad1.right_stick_x;
            double y = gamepad1.right_stick_y;

            double fl = -(x + y);
            double br = (x + y);
            double fr = -(y - x);
            double bl = -(y - x);

            //Mecanum drive code DON'T TOUCH THIS
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


            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(-bl);
            backRight.setPower(br);
            intake.setPower(-intakePower);
            roller.setPower(-rollerPower);

            //----------GAMEPAD 2------------

            if (gamepad2.y) {
                blocker.setPosition(0);
            } else if (gamepad2.b) {
                blocker.setPosition(0.2);
            }

            if (gamepad2.dpad_up) {
                hood.setPosition(0.75);
            } else if (gamepad2.dpad_down) {
                hood.setPosition(0.2);
            }

            // Auto centering.  If you are shooting short and a big far
            // then adjust the distance multiplier.
            double distanceMult = 1.0;
            if (gamepad1.dpad_down && gamepad2.right_bumper) {
                LLStatus status = limelight.getStatus();

                if (result.isValid()) {
                    double tx = result.getTx();
                    double ta = result.getTa();
                    setAlignmentRotatePower(limelight.getLatestResult(), -3.0);
                    distanceMult = 20.4 / (ta + 18);
                }
            } else if (gamepad1.dpad_down) {
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

            if (shootShort) {
                targetRPM = 2100 * distanceMult;

            } else if (shootLong) {
                targetRPM = 3100;

            } else {
                targetRPM = 2100;
            }

            if (gamepad2.x) {
                flywheel1.setPower(-1);
            }

            flywheel.setTargetRPM(-targetRPM);

            // Read velocity in ticks/sec

            double currentTPS = flywheel1.getVelocity();
            double targetTPS = (targetRPM / 60.0) * 28.0;

            if ((targetRPM > 0) && (Math.abs(currentTPS - targetTPS) < 10)) {
                gamepad2.rumble(100);
            }

            if (shootShort || shootLong && flywheel.isAtSpeed(3100, 60)) {
                blocker.setPosition(0.7);
            } else {
                blocker.setPosition(0);
            }



        }

    }
}