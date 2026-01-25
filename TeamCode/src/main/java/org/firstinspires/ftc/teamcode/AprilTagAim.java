package org.firstinspires.ftc.teamcode;

import org.json.JSONObject;
import com.qualcomm.robotcore.util.Range;

public class AprilTagAim {

    private PIDController turnPID;

    public AprilTagAim(double kP, double kI, double kD) {
        turnPID = new PIDController(kP, kI, kD);
    }

    // Returns rotation power needed to face the tag
    public double update(JSONObject ll) {

        // If no tag visible, return 0 (no auto-aim)
        if (ll.optDouble("tv", 0) == 0) {
            return 0;
        }

        // Horizontal offset from Limelight
        double tx = ll.optDouble("tx", 0);

        // PID tries to make tx → 0 (centered)
        double turnPower = turnPID.update(0, tx);

        // Limit rotation power
        return Range.clip(turnPower, -0.5, 0.5);
    }
}
