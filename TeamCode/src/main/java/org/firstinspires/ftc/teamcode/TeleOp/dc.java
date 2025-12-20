package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.util.Locale;

@TeleOp(name="ColorSensorTest")
public class dc extends LinearOpMode {
    ColorSensor colorsensor;

    private void InitColorSensor() {
        colorsensor = hardwareMap.colorSensor.get("colorsensor");
    }

    private double getHue(int r, int g, int b) {
        int max = Math.max(r, Math.max(g, b));
        int min = Math.min(r, Math.min(g, b));
        if (max == min) {
            return 0.0;
        }
        double chroma = max - min;
        double h;
        if (max == r) {
            h = (double)(g - b) / chroma;
        } else if (max == g) {
            h = (double)(b - r) / chroma + 2.0;
        } else {
            h = (double)(r - g) / chroma + 4.0;
        }
        h *= 60.0;
        if (h < 0) {
            h += 360.0;
        }
        return h;
    }

    private String detectColor() {
        int r = colorsensor.red();
        int g = colorsensor.green();
        int b = colorsensor.blue();
        int alpha = colorsensor.alpha();
        double h = getHue(r, g, b);


        if (alpha<100 && (h==150 || h==144)){return "Negru";}
        else if ((h > 215) || (alpha<100 && (h==160 || h==180))) {
            return "Magenta";
        } else if (h > 135 && h < 160) {
            return "Green";
        }
        else if ((h==210 || h==220 || h==225 || h==200) && alpha<100)//210 220 225 200
        return "magenta gaura";
        else
        {
            return "Negru";
        }
    }

    private void UpdateTelemetry() {
        int r = colorsensor.red();
        int g = colorsensor.green();
        int b = colorsensor.blue();
        int sum = r + g + b;
        double pr = sum > 0 ? (r * 100.0 / sum) : 0.0;
        double pg = sum > 0 ? (g * 100.0 / sum) : 0.0;
        double pb = sum > 0 ? (b * 100.0 / sum) : 0.0;
        double hue = getHue(r, g, b);

        telemetry.addData("Red %", String.format(Locale.US, "%.1f", pr));
        telemetry.addData("Green %", String.format(Locale.US, "%.1f", pg));
        telemetry.addData("Blue %", String.format(Locale.US, "%.1f", pb));
        telemetry.addData("Hue", String.format(Locale.US, "%.1f", hue));
        telemetry.addData("Alpha", colorsensor.alpha());
        telemetry.addData("Detected Color", detectColor());
        telemetry.update();
    }

    public void runOpMode()
    {
        InitColorSensor();
        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive() && !isStopRequested()) {
            UpdateTelemetry();
        }
    }
}