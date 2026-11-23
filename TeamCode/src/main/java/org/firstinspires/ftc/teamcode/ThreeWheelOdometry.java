package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.lynx.LynxModule;

@TeleOp(name = "3+1 Odometry + Mecanum Drive", group = "Test")
public class ThreeWheelOdometry extends LinearOpMode {

    // Motor & Encoder Names

    private static final String FRONT_LEFT_NAME  = "front_left";
    private static final String BACK_LEFT_NAME   = "back_left";   // Left Pod
    private static final String FRONT_RIGHT_NAME = "front_right"; // Right Pod
    private static final String BACK_RIGHT_NAME  = "back_right";  // Strafe Pod

    // Encoder Signs (+X Forward, +Y Left, +CCW)
    private static final int LEFT_POD_DIR   = -1;
    private static final int RIGHT_POD_DIR  = 1;
    private static final int STRAFE_POD_DIR = 1;

    // Robot Physical Constants (mm)
    private static final double TRACK_WIDTH_MM   = 220.0;
    private static final double STRAFE_OFFSET_MM = -150.0;


    // 3+1 Fusion & IMU Config
    private static final double ODOM_ROT_WEIGHT = 0.85;
    private static final double IMU_DRIFT_PULL  = 0.02;
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    // goBILDA 4-Bar Pod Specs (32mm wheel, 2000 CPR)
    private static final double MM_PER_TICK = (Math.PI * 32.0) / 2000.0;

    private double globalX = 0.0, globalY = 0.0, fusedHeadingRad = 0.0;
    private double prevImuCcwRad = 0.0;
    private int prevLeftTicks = 0, prevRightTicks = 0, prevStrafeTicks = 0;

    @Override
    public void runOpMode() {
        DcMotorEx frontLeft  = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_NAME);
        DcMotorEx backLeft   = hardwareMap.get(DcMotorEx.class, BACK_LEFT_NAME);
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_NAME);
        DcMotorEx backRight  = hardwareMap.get(DcMotorEx.class, BACK_RIGHT_NAME);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR)));

        telemetry.addLine("Ready. Press START.");
        telemetry.update();


        // ... inside runOpMode(), before waitForStart():
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        waitForStart();

        imu.resetYaw();
        prevLeftTicks   = backLeft.getCurrentPosition() * LEFT_POD_DIR;
        prevRightTicks  = frontRight.getCurrentPosition() * RIGHT_POD_DIR;
        prevStrafeTicks = backRight.getCurrentPosition() * STRAFE_POD_DIR;

        while (opModeIsActive()) {
            // 1. Mecanum TeleOp Drive
            double forward = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x; // Negated so +Y = Left
            double turn    = -gamepad1.right_stick_x;
            //double scale   = gamepad1.right_bumper ? 0.4 : 1.0;
            double scale = 0.6;  // Turn up to 100% later.

            double fl = (forward - strafe - turn) * scale;
            double bl = (forward + strafe - turn) * scale;
            double fr = (forward + strafe + turn) * scale;
            double br = (forward - strafe + turn) * scale;

            double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))));
            frontLeft.setPower(fl / max);
            backLeft.setPower(bl / max);
            frontRight.setPower(fr / max);
            backRight.setPower(br / max);

            // 2. 3+1 Odometry + IMU Fusion
            int currLeft   = backLeft.getCurrentPosition() * LEFT_POD_DIR;
            int currRight  = frontRight.getCurrentPosition() * RIGHT_POD_DIR;
            int currStrafe = backRight.getCurrentPosition() * STRAFE_POD_DIR;

            double dLeft   = (currLeft - prevLeftTicks) * MM_PER_TICK;
            double dRight  = (currRight - prevRightTicks) * MM_PER_TICK;
            double dStrafe = (currStrafe - prevStrafeTicks) * MM_PER_TICK;

            prevLeftTicks   = currLeft;
            prevRightTicks  = currRight;
            prevStrafeTicks = currStrafe;

            double currImuRad  = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double dThetaImu   = normalizeRadians(currImuRad - prevImuCcwRad);
            prevImuCcwRad      = currImuRad;

            double dThetaOdom  = (dRight - dLeft) / TRACK_WIDTH_MM;
            double dThetaFused = (ODOM_ROT_WEIGHT * dThetaOdom) + ((1.0 - ODOM_ROT_WEIGHT) * dThetaImu);

            // Robot-centric arc deltas (Chord lengths along local X and Y)
            double dForward  = (dLeft + dRight) / 2.0;
            double dLeftMove = dStrafe - (STRAFE_OFFSET_MM * dThetaFused);

            // Pose Exponential (SE(2) Arc Integration)
            double sinTerm;
            double cosTerm;
            if (Math.abs(dThetaFused) < 1e-6) {
                // Taylor series expansion avoids division by zero on straight lines
                sinTerm = 1.0 - (dThetaFused * dThetaFused) / 6.0;
                cosTerm = dThetaFused / 2.0;
            } else {
                sinTerm = Math.sin(dThetaFused) / dThetaFused;
                cosTerm = (1.0 - Math.cos(dThetaFused)) / dThetaFused;
            }

            // Convert arc lengths into local chord displacements
            double localDeltaX = (dForward * sinTerm) - (dLeftMove * cosTerm);
            double localDeltaY = (dForward * cosTerm) + (dLeftMove * sinTerm);

            // Rotate local chord displacements into the global frame using the starting heading
            double cosH = Math.cos(fusedHeadingRad);
            double sinH = Math.sin(fusedHeadingRad);
            globalX += (localDeltaX * cosH) - (localDeltaY * sinH);
            globalY += (localDeltaX * sinH) + (localDeltaY * cosH);

            // Update global heading after translation
            fusedHeadingRad = normalizeRadians(fusedHeadingRad + dThetaFused);
            fusedHeadingRad = normalizeRadians(fusedHeadingRad + IMU_DRIFT_PULL * normalizeRadians(currImuRad - fusedHeadingRad));

            if (gamepad1.a) {
                globalX = 0.0;
                globalY = 0.0;
                imu.resetYaw();
                prevImuCcwRad = 0.0;
                fusedHeadingRad = 0.0;
            }

            // 3. Telemetry
            telemetry.addData("X (Forward)", "%.1f mm (%.2f in)", globalX, globalX / 25.4);
            telemetry.addData("Y (Left)",    "%.1f mm (%.2f in)", globalY, globalY / 25.4);
            telemetry.addData("3+1 Heading", "%.2f° (IMU: %.2f°)", Math.toDegrees(fusedHeadingRad), Math.toDegrees(currImuRad));
            telemetry.addData("Ticks (L|R|S)", "%d | %d | %d", currLeft, currRight, currStrafe);
            telemetry.addLine("[RB] Slow Mode | [A] Zero Pose");
            telemetry.addData("dTheta: (I,O,F)", "%.4f° | %.4f° | %.4f°", dThetaImu, dThetaOdom, dThetaFused);
            telemetry.update();
        }
    }

    private double normalizeRadians(double angle) {
        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}