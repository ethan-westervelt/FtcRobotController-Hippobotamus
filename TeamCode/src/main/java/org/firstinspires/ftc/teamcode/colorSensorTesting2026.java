package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp

public class colorSensorTesting2026 extends LinearOpMode {

    private RevColorSensorV3 colorSensor;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");

        waitForStart();

        while (opModeIsActive()) {

            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            double hue = JavaUtil.colorToHue(colors.toColor());

            String color = "whatever you want it to be";
            double alpha = colors.alpha;
            double red = colors.red;
            double blue = colors.blue;
            double green = colors.green;


            if (alpha > 0.7) {
                if (hue > 300 || hue < 60) {
                    color = "red";
                } else if (hue > 60 && hue < 180) {
                    color = "yellow";
                } else if (hue > 180 && hue < 300) {
                    color = "blue";
                }
            } else {
                color = "I'm not sure";
            }

                telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);
            telemetry.addData("Alpha", "%.3f", colors.alpha);
            telemetry.addData("Hue", "%.3f", hue);
            telemetry.addData("Color", color);
            telemetry.update();
        }
    }
}
