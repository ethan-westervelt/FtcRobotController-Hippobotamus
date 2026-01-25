package org.firstinspires.ftc.teamcode22482;

import org.json.JSONObject;
import java.net.URL;
import java.util.Scanner;

public class Limelight {

    public static JSONObject get() {
        try {
            URL url = new URL("http://limelight.local:5807/lljson");
            Scanner s = new Scanner(url.openStream()).useDelimiter("\\A");
            String json = s.hasNext() ? s.next() : "{}";
            return new JSONObject(json);
        } catch (Exception e) {
            return new JSONObject();
        }
    }


}


