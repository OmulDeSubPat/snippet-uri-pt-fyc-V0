package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.util.Locale;

@TeleOp(name="ColorSensorAlgoritm")
public class TestAlgoritmVector extends LinearOpMode {

    ColorSensor colorsensor;

    int[] last5 = new int[5];
    int index = 0;

    private void pushColor(int val) {
        last5[index] = val;
        index = (index + 1) % 5;
    }

    private void resetBuffer() {
        for (int i = 0; i < 5; i++) last5[i] = 0;
        index = 0;
    }

    private int getMajority() {
        int count0 = 0, count1 = 0, count2 = 0;
        for (int v : last5) {
            if (v == 0) count0++;
            else if (v == 1) count1++;
            else if (v == 2) count2++;
        }

        if (count1 >= 3 && count1 > count2) return 1;
        if (count2 >= 3 && count2 > count1) return 2;

        return 0;
    }

    int[] slots = new int[3];
    int slotIndex = 0;

    private void resetSlots() {
        slots[0] = 0;
        slots[1] = 0;
        slots[2] = 0;
        slotIndex = 0;
    }

    private void InitColorSensor() {
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

    private int detectColor() {
        int r = colorsensor.red();
        int g = colorsensor.green();
        int b = colorsensor.blue();
        int alpha = colorsensor.alpha();
        double h = getHue(r, g, b);

        if (alpha < 100 && (h == 150 || h == 144)) return 0;
        else if ((h > 215) || (alpha < 100 && (h == 160 || h == 180))) return 2;
        else if (h > 135 && h < 160) return 1;
        else if ((h == 210 || h == 220 || h == 225 || h == 200) && alpha < 100) return 2;

        return 0;
    }

    private void UpdateTelemetry(int finalColor) {
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
        telemetry.addData("Raw", detectColor());
        telemetry.addData("Majority", finalColor);

        telemetry.addData("Slot 1", slots[0]);
        telemetry.addData("Slot 2", slots[1]);
        telemetry.addData("Slot 3", slots[2]);
        telemetry.addData("Next Slot Index", slotIndex);

        telemetry.update();
    }

    @Override
    public void runOpMode() {
        InitColorSensor();
        resetBuffer();
        resetSlots();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.ps) {
                resetBuffer();
                resetSlots();
                telemetry.addData("RESET", "Buffer + Slot-uri resetate!");
                telemetry.update();
                sleep(300);
            }

            int detected = detectColor();
            pushColor(detected);
            int finalColor = getMajority();
            UpdateTelemetry(finalColor);

            if ((finalColor == 1 || finalColor == 2) && slotIndex < 3) {
                slots[slotIndex] = finalColor;
                slotIndex++;
                //resetBuffer();
                while (detectColor() != 0 && opModeIsActive()) {
                    sleep(500);
                }
            }
        }
    }
}
