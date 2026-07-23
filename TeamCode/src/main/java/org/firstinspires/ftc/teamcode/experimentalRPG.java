//DON'T MESS WITH THIS IT'S MAGIC
package org.firstinspires.ftc.teamcode;


// We need to import external code (code someone else wrote) to make the robot run
//DON'T CHANGE ANY OF THIS, OR ELSE THINGS WON'T WORK!!!

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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

    double turretRotate = 0;

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


    double ttx = 0;

    double calcTurretAlignmentPower(LLResult result) {

        if (result.isValid()) {
            ttx = 0.7 * result.getTx() + 0.3 * ttx;
        } else {
            ttx = 0.9 * ttx;
        }

        double turretPower = 0;
        if (Math.abs(ttx) > 0.2) {
            turretPower = -ttx * 0.025;//output;
        }
        return (turretPower);
    }

    // Declarations for the objects that represent the motors/servos:
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor turret;
    private DcMotor roller;
    private DcMotorEx flywheel1;
    private Servo blocker;
    private Servo hood;
    private DcMotor intake;
    private ElapsedTime timer = new ElapsedTime();
    private Limelight3A limelight;
    PIDController turretPID = new PIDController(0.03, 0, 0.0001);

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
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        roller = hardwareMap.get(DcMotor.class, "roller");

        // You need this many ticks of the motor to get one degree of turret rotation
        double turretEncPerDeg = -3.78;

        //LIMELIGHT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(50);
        limelight.start();

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
        double rollerPower;
        double targetRPM = 100;//2000;

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

        PIDController flywheelPID = new PIDController(0.4, 0.0000, 0.0003);
        //flywheel1.setVelocityPIDFCoefficients();

        while (opModeIsActive()) {

            // -------------------------------
            // ----------GAMEPAD 1 ------------
            // -------------------------------

            // Forward (y) and strafe (x)
            double x = -gamepad1.right_stick_x;
            double y = gamepad1.right_stick_y;

            // Trigger is fast spin, bumpers are slow spin
            if (gamepad1.right_trigger > 0) {
                rotate = -1;
            } else if (gamepad1.left_trigger > 0) {
                rotate = 1;
            } else if (gamepad1.right_bumper) {
                rotate = -0.3;
            } else if (gamepad1.left_bumper) {
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
            if (Math.abs(fl) > maxPower) {
                maxPower = Math.abs(fl);
            }
            if (Math.abs(bl) > maxPower) {
                maxPower = Math.abs(bl);
            }
            if (Math.abs(fr) > maxPower) {
                maxPower = Math.abs(fr);
            }
            if (Math.abs(br) > maxPower) {
                maxPower = Math.abs(br);
            }

            if (maxPower > 1) {
                maxPower = 1;
            }

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
            // Positive power means counterclockwise.
            LLResult result = limelight.getLatestResult();
            double currentPos = turret.getCurrentPosition();
            double turretPos = currentPos / turretEncPerDeg;

            turretPower = 0;

            //Manual turret control
            if (gamepad2.left_trigger > 0.1) {
                turretPower = 0.6 * gamepad2.left_trigger;
            }
            if (gamepad2.right_trigger > 0.1) {
                turretPower = -0.6 * gamepad2.right_trigger;
            }

            // Turret auto centering
            // If you're holding down the button, try to use auto recentering.
            // If you don't find it then it falls back to what you're doing manually.
            if (gamepad2.y) {
                if (result.isValid()) {
                    turretPower = calcTurretAlignmentPower(result);
                }
            }
            telemetry.addData("turretPower_PRE", turretPower);


            currentPos = turret.getCurrentPosition();
            turretPos = currentPos / turretEncPerDeg;

            // If you're too far counterclockwise you can only have negative power
            if (turretPos < -105) {
                turretPower = Math.min(0, turretPower);
            }
            // Likewise, if you're too far clockwise, you can only have positive power
            if (turretPos > 105) {
                turretPower = Math.max(0, turretPower);
            }
            turret.setPower(turretPower);

            telemetry.addData("turretPos", turret.getCurrentPosition());
            telemetry.addData("turretPower", turretPower);
            telemetry.addData("turretTx", ttx);
            telemetry.addData("rotatePower", turretRotate);

            if (result != null && result.isValid()) {//
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            //intake power
            if (gamepad2.left_stick_y != 0) {
                rollerPower = gamepad2.left_stick_y;
                intakePower = gamepad2.left_stick_y;
            } else if (gamepad2.right_stick_y != 0) {
                rollerPower = gamepad2.right_stick_y / 0.75;
                intakePower = gamepad2.right_stick_y / 0.75;
            } else {
                rollerPower = 0;
                intakePower = 0;
            }

            intake.setPower(intakePower);
            roller.setPower(rollerPower);

            if (gamepad2.dpad_up) {
                hood.setPosition(0.75);
            } else if (gamepad2.dpad_down) {
                hood.setPosition(0.2);
            }


            // Auto centering.  If you are shooting short and a big far
            // then adjust the distance multiplier.
            double ta = result.getTa();
            double distanceMult = 8.2 / (ta + 6.2); // 20.4, 18
            telemetry.addData("Hood position: ", hood.getPosition());
            telemetry.addData("distanceMult", distanceMult);
            telemetry.addData("Target speed", (targetRPM / 60) * 28);
            telemetry.addData("Actual speed", flywheel1.getVelocity());

            if (gamepad2.dpad_up) {
                targetRPM = targetRPM + 5;//20;
                sleep(250);
            } else if (gamepad2.dpad_down) {
                targetRPM = targetRPM - 5;//20;
                sleep(250);
            }

            telemetry.addData("targetRPM", targetRPM);
            telemetry.update();

            boolean shootShort = gamepad2.left_bumper;
            boolean shootLong = gamepad2.right_bumper;
            /*
            if (shootShort) {
                targetRPM = 2500;// * distanceMult;
            } else if (shootLong) {
                targetRPM = 3500;//dont know if we need the distance adapter here
            } else {
                targetRPM = 2500;
            }
            */
            double target = targetRPM;
            //double currentTPS = flywheel1.getVelocity();   // ticks per second
            double currentrps = flywheel1.getVelocity(AngleUnit.RADIANS);
            //double targetTPS = (target / 60.0) * 28.0;  // convert RPM → ticks/sec

            //double pidOutput = flywheelPID.update(targetTPS, currentTPS);
            //pidOutput = Range.clip(pidOutput, -1, 1); //eliminates any powers that are over or under 1 and -1
            //double kF = 0.00042;
            //double output = pidOutput + kF * targetTPS;

            double targetrps = 2.0*3.141592*(targetRPM/60);
            flywheel1.setVelocity(targetrps, AngleUnit.RADIANS);

            //flywheel1.setPower(output);

            telemetry.addData("Target RPM", targetRPM); //targetTPS);
            //telemetry.addData("Current TPS", currentTPS);
            telemetry.addData("Current RPM", currentrps*60/2.0/3.141592); //2.0*3.141592*currentTPS/28.0);
            //telemetry.addData("PID Output", pidOutput);
            // telemetry.update();

            //PIDFCoefficients pidf = flywheel1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            PIDFCoefficients pidfNew = new PIDFCoefficients(300, 1, 0.0, 0.0); // default = 10;3;0;0
            flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

            //telemetry.addData("P", pidfNew.p);
            //telemetry.addData("I", pidfNew.i);
            //telemetry.addData("D", pidfNew.d);
            //telemetry.addData("F", pidfNew.f);

            //if ((targetRPM > 0) && (Math.abs(currentTPS - targetTPS) < 10)) {
            //    gamepad2.rumble(100);
            //}

            hood.setPosition(0.75);

            if (shootShort) { //flywheel.isAtSpeed(2350, 200)
                blocker.setPosition(0.75);

            } else if (shootLong) {
                hood.setPosition(0.75);
                blocker.setPosition(0.75);
            } else {
                blocker.setPosition(0.75);
                }


        }
    }
}