package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensorProcessor {
    //variabile colorate 😉
    //TODO//sa fie folosite corect
    int[] last5 = new int[5];
    int index = 0;
    ColorSensor colorsensor;

    public ColorSensorProcessor(HardwareMap hardwareMap) {
        colorsensor = hardwareMap.colorSensor.get("colorsensor");
    }

    private double getHue(int r, int g, int b) {
        int max = Math.max(r, Math.max(g, b));
        int min = Math.min(r, Math.min(g, b));
        if (max == min) return 0.0;
        double chroma = max - min;
        double h;
        if (max == r) h = (double)(g - b) / chroma;
        else if (max == g) h = (double)(b - r) / chroma + 2.0;
        else h = (double)(r - g) / chroma + 4.0;
        h *= 60.0;
        if (h < 0) h += 360.0;
        return h;
    }

    public int processColorSensor() {
        int r = colorsensor.red();
        int g = colorsensor.green();
        int b = colorsensor.blue();
        int alpha = colorsensor.alpha();

        double h = getHue(r, g, b);

        int detected;
        //nebunia mea si alu fane
        if (alpha < 100 && (h == 150 || h == 144)) detected = 0;
        else if ((h > 215) || (alpha < 100 && (h == 160 || h == 180))) detected = 2;
        else if (h > 135 && h < 160) detected = 1;
        else if ((h == 210 || h == 220 || h == 225 || h == 200) && alpha < 100) detected = 2;
        else detected = 0;

        last5[index] = detected;
        index = (index + 1) % 5;

        int  count1 = 0, count2 = 0;
        for (int v : last5) {
            if (v == 1) count1++;
            else if (v == 2) count2++;
        }

        int finalColor = 0;
        if (count1 >= 3 && count1 > count2) finalColor = 1;
        else if (count2 >= 3 && count2 > count1) finalColor = 2;

        return finalColor;
    }

   /* public void updateTelemetry(Telemetry telemetry) {
        int finalColor = processColorSensor();
        telemetry.addData("Final Color", finalColor);
        telemetry.update();
    }*/
}