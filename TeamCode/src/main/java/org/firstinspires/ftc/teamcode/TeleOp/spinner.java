package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="ColorDetection+cevaspinner")
public class spinner extends LinearOpMode {

    int[] last5Sensor1 = new int[5];
    int[] last5Sensor2 = new int[5];
    int[] last5Sensor3 = new int[5];
    int indexSensor1 = 0;
    int indexSensor2 = 0;
    int indexSensor3 = 0;

    int[] slots = new int[3];

    int lastStableColorSensor1 = 0;
    int lastStableColorSensor2 = 0;
    int lastStableColorSensor3 = 0;

    boolean spinnerBusy = false;

    ColorSensor colorsensorSLot1;
    ColorSensor colorsensorSLot2;
    ColorSensor colorsensorSLot3;

    DcMotorEx spinner;


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

    private int smekerie(ColorSensor colorSensor) {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        int alpha = colorSensor.alpha();
        double h = getHue(r, g, b);
        int detected;
        if (alpha < 100 && (h == 150 || h == 144)) detected = 0;
        else if ((h > 215) || (alpha < 100 && (h == 160 || h == 180))) detected = 2;
        else if (h > 135 && h < 160) detected = 1;
        else if ((h == 210 || h == 220 || h == 225 || h == 200) && alpha < 100) detected = 2;
        else detected = 0;
        return detected;
    }

    private int processSingleSensor(ColorSensor sensor, int[] last5, int index) {
        int detected = smekerie(sensor);
        last5[index] = detected;
        index = (index + 1) % 5;
        int count1 = 0;
        int count2 = 0;
        for (int v : last5) {
            if (v == 1) count1++;
            else if (v == 2) count2++;
        }
        int finalColor = 0;
        if (count1 >= 3 && count1 > count2) finalColor = 1;
        else if (count2 >= 3 && count2 > count1) finalColor = 2;
        return finalColor;
    }

    private void updateAllSlots() {
        slots[0] = processSingleSensor(colorsensorSLot1, last5Sensor1, indexSensor1);
        indexSensor1 = (indexSensor1 + 1) % 5;
        slots[1] = processSingleSensor(colorsensorSLot2, last5Sensor2, indexSensor2);
        indexSensor2 = (indexSensor2 + 1) % 5;
        slots[2] = processSingleSensor(colorsensorSLot3, last5Sensor3, indexSensor3);
        indexSensor3 = (indexSensor3 + 1) % 5;
    }

    private void colorDrivenSpinnerLogic() {
        boolean newColorDetected = false;
        if (slots[0] != 0 && slots[0] != lastStableColorSensor1) newColorDetected = true;
        if (slots[1] != 0 && slots[1] != lastStableColorSensor2) newColorDetected = true;
        if (slots[2] != 0 && slots[2] != lastStableColorSensor3) newColorDetected = true;
        if (newColorDetected && !spinnerBusy) {
            spinnerBusy = true;
           /* spinner.setTargetPosition(spinner.getCurrentPosition() + 560);// aici ar veni logica de PID din codu mare
            spinner.setPower(0.5);*/
            //vedem epiric cat dureaza exact sa se dea ejector ul sa se duca sus si jos sau bagam un isBusy
            //ne putem folosi de update urile astea pana stim ca toate slot urile sunt ocupate apoi updatam de abia in outtake
        }
        lastStableColorSensor1 = slots[0];
        lastStableColorSensor2 = slots[1];
        lastStableColorSensor3 = slots[2];
        if (spinnerBusy && !spinner.isBusy()) {
            spinner.setPower(0);//la fel si aici
            spinnerBusy = false;
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Slot 1", slots[0]);
        telemetry.addData("Slot 2", slots[1]);
        telemetry.addData("Slot 3", slots[2]);
        telemetry.addData("Pregatit de lanasare", spinnerBusy);
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        colorsensorSLot1 = hardwareMap.colorSensor.get("ColorSensor1");
        colorsensorSLot2 = hardwareMap.colorSensor.get("ColorSensor2");
        colorsensorSLot3 = hardwareMap.colorSensor.get("ColorSensor3");
        spinner = hardwareMap.get(DcMotorEx.class, "spinner");
        waitForStart();
        while (opModeIsActive()) {
            updateAllSlots();
            colorDrivenSpinnerLogic();
            updateTelemetry();
            idle();
        }
    }
}
