package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@TeleOp(name = "ColorDetection+cevaspinner")
public class spinner extends LinearOpMode {

    // Timers
    ElapsedTime pidTimer = new ElapsedTime();
    private ElapsedTime sortTimer = new ElapsedTime();

    // Color sensor memory
    int[] last5Sensor1 = new int[5];
    int indexSensor1 = 0;

    // Spinner slots
    int[] slots = new int[3];
    int[] totem = {1, 2, 1};

    // Color tracking
    int lastStableColorSensor1 = 0;
    int lastStableColorSensor2 = 0;
    int lastStableColorSensor3 = 0;

    // Spinner state
    boolean spinnerBusy = false;
    boolean sortingActive = false;
    int lastSpinnerStep = 0;

    // Hardware
    ColorSensor colorsensorSLot1;
    // ColorSensor colorsensorSLot2;
    // ColorSensor colorsensorSLot3;
    DcMotorEx spinner;
    DcMotor intake;
    DcMotor tureta;
    Servo ejector;
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;

    // Spinner PID
    static final double TICKS_PER_REV = 384.5;
    static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;
    double P = 0.0101;
    double I = 0.0000;
    double D = 0.0015;
    double integralSum = 0;
    double lastError = 0;
    int targetTicks = 0;

    // Color detection timing
    long colorStartTime = 0;
    boolean colorPending = false;

    private void InitWheels() {
        front_left = hardwareMap.dcMotor.get("lf");
        front_right = hardwareMap.dcMotor.get("rf");
        back_left = hardwareMap.dcMotor.get("lr");
        back_right = hardwareMap.dcMotor.get("rr");

        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private int getSpinnerStep() {
        int stepTicks = (int) (120 * TICKS_PER_DEGREE);
        return Math.round((float) spinner.getCurrentPosition() / stepTicks);
    }

   /* private void updateSpinnerMemoryFromEncoder() {
        int currentStep = getSpinnerStep();
        int delta = currentStep - lastSpinnerStep;

        if (delta == 1) {
            // moved +120° (right)
            slots = rotateRight(slots);
        } else if (delta == -1) {
            // moved -120° (left)
            slots = rotateLeft(slots);
        }

        if (delta != 0) {
            // update the color under the sensor
            slots[0] = processSingleSensor(colorsensorSLot1, last5Sensor1, indexSensor1);
            indexSensor1 = (indexSensor1 + 1) % 5;
        }

        lastSpinnerStep = currentStep;
    }
*/
    private void SetWheelsPower() {
        double left_x = gamepad1.left_stick_x;
        double left_y = -gamepad1.left_stick_y; // forward is negative
        double right_x = gamepad1.right_stick_x;

        double front_left_pw = left_y + left_x + right_x;
        double back_left_pw = left_y - left_x + right_x;
        double front_right_pw = left_y - left_x - right_x;
        double back_right_pw = left_y + left_x - right_x;

        // Normalize so no motor power exceeds 1.0
        double max = Math.max(Math.abs(front_left_pw),
                Math.max(Math.abs(back_left_pw),
                        Math.max(Math.abs(front_right_pw), Math.abs(back_right_pw))));
        if (max > 1.0) {
            front_left_pw /= max;
            back_left_pw /= max;
            front_right_pw /= max;
            back_right_pw /= max;
        }

        front_left.setPower(front_left_pw);
        back_left.setPower(back_left_pw);
        front_right.setPower(front_right_pw);
        back_right.setPower(back_right_pw);
    }

    private double getHue(int r, int g, int b) {
        int max = Math.max(r, Math.max(g, b));
        int min = Math.min(r, Math.min(g, b));
        if (max == min) return 0.0;

        double chroma = max - min;
        double h;

        if (max == r) h = (double) (g - b) / chroma;
        else if (max == g) h = (double) (b - r) / chroma + 2.0;
        else h = (double) (r - g) / chroma + 4.0;

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

        int count1 = 0; // green
        int count2 = 0; // purple

        for (int v : last5) {
            if (v == 1) count1++;
            else if (v == 2) count2++;
        }

        int finalColor = 0;
        if (count1 >= 3 && count1 > count2) finalColor = 1;
        else if (count2 >= 3 && count2 > count1) finalColor = 2;

        return finalColor;
    }

    //updateAllSlots face update first slot
    private void updateAllSlots() {
        slots[0] = processSingleSensor(colorsensorSLot1, last5Sensor1, indexSensor1);
        indexSensor1 = (indexSensor1 + 1) % 5;
        // slots[1] = processSingleSensor(colorsensorSLot2, last5Sensor2, indexSensor2);
        // slots[2] = processSingleSensor(colorsensorSLot3, last5Sensor3, indexSensor3);
    }

    private boolean spinnerIsFull() {
        return slots[0] != 0 && slots[1] != 0 && slots[2] != 0;
    }

    private void colorDrivenSpinnerLogic() {
        boolean newColorDetected = false;
        if (slots[0] != 0 && (slots[0] != lastStableColorSensor1)) {
            newColorDetected = true;
            lastStableColorSensor1 = slots[0];
        }

        if (newColorDetected && !colorPending) {
            colorStartTime = System.currentTimeMillis();
            colorPending = true;
        }

        if (colorPending && System.currentTimeMillis() - colorStartTime >= 50) {
            targetTicks += (int) (120 * TICKS_PER_DEGREE);
            colorPending = false;

            slots = rotateRight(slots);

            // IMPORTANT: allow same-color next piece to trigger
            lastStableColorSensor1 = 0;
        }

    }

    private int[] rotateRight(int[] a) {
        return new int[]{a[2], a[0], a[1]};
    }

    private int[] rotateLeft(int[] a) {
        return new int[]{a[1], a[2], a[0]};
    }

    private String arrayToString(int[] a) {
        return "[" + a[0] + ", " + a[1] + ", " + a[2] + "]";
    }

    private void Sort() {
        if (!gamepad1.yWasPressed()) return;

        sortingActive = true;
        sortTimer.reset();
        int maxAttempts = 3;

        for (int i = 0; i < maxAttempts; i++) {
            while (sortTimer.milliseconds() < 10 && opModeIsActive()) idle();
            sortTimer.reset();

            if (Arrays.equals(slots, totem)) {
                sortingActive = false;
                return;
            }

            int[] right = rotateRight(slots);
            if (Arrays.equals(right, totem)) {
                targetTicks += (int) (120 * TICKS_PER_DEGREE);
                slots = right.clone();
                continue;
            }

            int[] left = rotateLeft(slots);
            if (Arrays.equals(left, totem)) {
                targetTicks -= (int) (120 * TICKS_PER_DEGREE);
                slots = left.clone();
                continue;
            }

            // Default rotation if no match
            targetTicks += (int) (120 * TICKS_PER_DEGREE);
            slots = right.clone();
        }

        sortingActive = false;
    }

    private void updateTelemetry() {
        telemetry.addData("Slot 1", slots[0]);
        telemetry.addData("Slot 2", slots[1]);
        telemetry.addData("Slot 3", slots[2]);
        telemetry.addData("Pregatit de lansare", spinnerBusy);
        telemetry.addData("TargetTicks", targetTicks);
        telemetry.addData("SpinnerPos", spinner.getCurrentPosition());
        telemetry.addData("ColorPending", colorPending);
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        InitWheels();

        colorsensorSLot1 = hardwareMap.colorSensor.get("Color1");
        // colorsensorSLot2 = hardwareMap.colorSensor.get("Color2");
        // colorsensorSLot3 = hardwareMap.colorSensor.get("Color3");

        spinner = hardwareMap.get(DcMotorEx.class, "spinner");
        intake = hardwareMap.get(DcMotor.class, "intake");
        // ejector = hardwareMap.get(Servo.class, "ejector");
        tureta = hardwareMap.get(DcMotor.class, "tureta");

        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        tureta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        pidTimer.reset();

        while (opModeIsActive()) {

            // Intake control
            if (gamepad1.dpadDownWasPressed()) intake.setPower(-1);
            if (gamepad1.psWasPressed()) intake.setPower(0);

            // PID for spinner
            double currentPos = spinner.getCurrentPosition();
            double error = targetTicks - currentPos;
            integralSum += error;
            double derivative = error - lastError;
            lastError = error;
            double pidOutput = error * P + integralSum * I + derivative * D;
            pidOutput = Math.max(-0.5, Math.min(0.5, pidOutput));
            spinner.setPower(pidOutput);

            // Manual spinner control
            if (gamepad1.dpadRightWasPressed()) {
                targetTicks += (int) (120 * TICKS_PER_DEGREE);
                slots = rotateRight(slots);
                lastStableColorSensor1 = 0;
            }

            if (gamepad1.dpadLeftWasPressed()) {
                targetTicks -= (int) (120 * TICKS_PER_DEGREE);
                slots = rotateLeft(slots);
                lastStableColorSensor1 = 0;
            }

            // Update colors
            if (!sortingActive && !spinnerIsFull()) {
                updateAllSlots();
                colorDrivenSpinnerLogic();
            }

            updateTelemetry();
            SetWheelsPower();
            Sort();
            //updateSpinnerMemoryFromEncoder();

            idle();



        }
    }
}
