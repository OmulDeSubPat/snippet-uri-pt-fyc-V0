package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "&TeleOpMain")
public class TeleOpMain extends LinearOpMode {

    int[] last5Sensor1 = new int[5];
    int[] last5Sensor2 = new int[5];
    int[] last5Sensor3 = new int[5];

    int indexSensor1 = 0;
    int indexSensor2 = 0;
    int indexSensor3 = 0;

    int[] slots = new int[3];
    int[] totem = {2, 1, 2};

    int Color1 = 0;
    int Color2 = 0;
    int Color3 = 0;

    ColorSensor colorsensorSLot1;
    ColorSensor colorsensorSLot2;
    ColorSensor colorsensorSLot3;
    DcMotorEx spinner;
    DcMotor intake;
    DcMotorEx tureta;
    Servo ejector;
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;
    DcMotor flywheel;

    static final double TICKS_PER_REV = 384.5;
    double P = 0.0101;
    double I = 0.0001;
    double D = 0.0015;
    double integralSum = 0;

    double lastError = 0;
    double targetTicks = 0;

    boolean step1Done = false;
    boolean step2Done = false;
    boolean step3Done = false;
    boolean step4Done = false;
    boolean step5Done = false;
    boolean step6Done = false;
    boolean step7Done = false;
    boolean step8Done = false;
    boolean step9Done = false;
    boolean step10Done = false;

    Limelight3A limelight;
    IMU imu;
    int outtakeBaseSlot = 0;

    double DEG_PER_TICK = 360.0 / 383.6;
    boolean flywheelOn = false;
    TouchSensor limitswitch;
    boolean intakeMode = false;
    boolean outtakeMode = false;
    private ElapsedTime spinnerTimeout = new ElapsedTime();
    private ElapsedTime outtakeTimeout = new ElapsedTime();
    double ejectorDown = 0.285;
    double ejectorUp = 0.005;

    static final int TOTAL_POSITIONS = 6;
    static final int VALID_SLOTS = 5;
    static final double TICKS_PER_POS = TICKS_PER_REV / TOTAL_POSITIONS;
    static final double HOME_TO_SLOT0 = TICKS_PER_POS * 1.0;

    boolean homed = false;

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

    private void InitDc() {
        spinner = hardwareMap.get(DcMotorEx.class, "spinner");
        intake = hardwareMap.get(DcMotor.class, "intake");
        tureta = hardwareMap.get(DcMotorEx.class, "tureta");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        limitswitch = hardwareMap.get(TouchSensor.class, "limitswitch");

        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        tureta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tureta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tureta.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    private void InitServo() {
        ejector = hardwareMap.get(Servo.class, "ejector");
        ejector.setPosition(0.285);
    }

    private int getCurrentSlot() {
        long pos = Math.round((spinner.getCurrentPosition() - HOME_TO_SLOT0) / TICKS_PER_POS);
        return Math.floorMod(pos, VALID_SLOTS);
    }

    private void goToSlot(int slot) {
        slot = Math.floorMod(slot, VALID_SLOTS);
        targetTicks = HOME_TO_SLOT0 + slot * TICKS_PER_POS;
    }

    private void InitLL() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(orientation));
    }

    private void InitAux() {
        colorsensorSLot1 = hardwareMap.colorSensor.get("Color1");
        colorsensorSLot2 = hardwareMap.colorSensor.get("Color2");
        colorsensorSLot3 = hardwareMap.colorSensor.get("Color3");
    }

    private void SetWheelsPower() {
        double left_x = gamepad2.left_stick_x;
        double left_y = -gamepad2.left_stick_y;
        double right_x = gamepad2.right_stick_x;

        double front_left_pw = left_y + left_x + right_x;
        double back_left_pw = left_y - left_x + right_x;
        double front_right_pw = left_y - left_x - right_x;
        double back_right_pw = left_y + left_x - right_x;

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
        else if (h > 135 && h < 160 && alpha > 100) detected = 1;
        else if ((h == 140 || h == 145) && alpha == 43) detected = 0;
        else if (h > 135 && h < 160 && alpha > 60) detected = 1;
        else if ((h == 210 || h == 220 || h == 225 || h == 200) && alpha < 100) detected = 2;
        else detected = 0;
        return detected;
    }

    private int CuloareFinala(ColorSensor sensor, int[] last5, int index) {
        last5[index] = smekerie(sensor);
        int count1 = 0, count2 = 0;
        for (int v : last5) {
            if (v == 1) count1++;
            else if (v == 2) count2++;
        }
        if (count1 >= 3) return 1;
        if (count2 >= 3) return 2;
        return 0;
    }

    private void updateCulori() {
        Color1 = CuloareFinala(colorsensorSLot1, last5Sensor1, indexSensor1);
        indexSensor1 = (indexSensor1 + 1) % 5;
        Color2 = CuloareFinala(colorsensorSLot2, last5Sensor2, indexSensor2);
        indexSensor2 = (indexSensor2 + 1) % 5;
        Color3 = CuloareFinala(colorsensorSLot3, last5Sensor3, indexSensor3);
        indexSensor3 = (indexSensor3 + 1) % 5;
    }

    private void flywheelLogic() {
        if (gamepad1.shareWasPressed()) {
            flywheelOn = !flywheelOn;
            if (flywheelOn) flywheel.setPower(0.45);
            else flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void servoLogic() {
        if (gamepad1.optionsWasReleased()) ejector.setPosition(0.285);
        else if (gamepad1.optionsWasPressed()) ejector.setPosition(0.005);
    }

    private void pidSPinnerLogic() {
        if (!homed) {
            spinner.setPower(0);
            return;
        }
        double current = spinner.getCurrentPosition();
        double error = targetTicks - current;
        if (Math.abs(error) < 2) {
            spinner.setPower(0);
            integralSum = 0;
            return;
        }
        integralSum += error;
        double derivative = error - lastError;
        lastError = error;
        double power = error * P + integralSum * I + derivative * D;
        power = Math.max(-0.5, Math.min(0.5, power));
        spinner.setPower(power);
    }

    private void protectBlindSpot() {
        if (Math.abs(targetTicks) < TICKS_PER_POS * 0.5) goToSlot(0);
    }

    private boolean spinnerFull() {
        return (Color1 != 0 && Color2 != 0 && Color3 != 0);
    }

    private void colorDrivenSpinnerLogic() {
        if (Color1 != 0 && !spinnerFull()) {
            if (spinnerTimeout.milliseconds() >= 1000) {
                int nextSlot = getCurrentSlot() + 2;
                goToSlot(nextSlot);
                spinnerTimeout.reset();
            }
        }
    }

    private void updateTelemetry() {
        if (outtakeMode) {
            telemetry.addData("Slot 1", slots[0]);
            telemetry.addData("Slot 2", slots[1]);
            telemetry.addData("Slot 3", slots[2]);
        }
        if (intakeMode) {
            telemetry.addData("Slot 1", Color1);
            telemetry.addData("Slot 2", Color2);
            telemetry.addData("Slot 3", Color3);
        }
        telemetry.addData("timp_intake", spinnerTimeout.time());
        telemetry.addData("timp_outtake", outtakeTimeout.time());
        telemetry.update();
    }

    private void homeSpinner() {
        spinner.setPower(-0.2);
        if (limitswitch.isPressed()) {
            spinner.setPower(0);
            spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            integralSum = 0;
            lastError = 0;
            targetTicks = HOME_TO_SLOT0; // move away from blind spot
            homed = true;
        }
    }

    private void runOuttake() {
        slots[0] = Color1;
        slots[1] = Color2;
        slots[2] = Color3;
        double t = outtakeTimeout.milliseconds();
        int baseSlot = outtakeBaseSlot;

        if (t >= 10 && !step1Done) { goToSlot(baseSlot+1); step1Done = true; }
        if (t >= 1000 && !step2Done) { ejector.setPosition(ejectorUp); step2Done = true; }
        if (t >= 2000 && !step3Done) { ejector.setPosition(ejectorDown); step3Done = true; }
        if (t >= 3500 && !step4Done) { goToSlot(baseSlot + 2); step4Done = true; }
        if (t >= 5000 && !step5Done) { ejector.setPosition(ejectorUp); step5Done = true; }
        if (t >= 6000 && !step6Done) { ejector.setPosition(ejectorDown); step6Done = true; }
        if (t >= 7000 && !step7Done) { goToSlot(baseSlot + 4); step7Done = true; }
        if (t >= 8500 && !step8Done) { ejector.setPosition(ejectorUp); step8Done = true; }
        if (t >= 9500 && !step9Done) { ejector.setPosition(ejectorDown); step9Done = true; }
        if (t >= 10000 && !step10Done) { outtakeMode = false; step10Done = true; }
    }

    @Override
    public void runOpMode() {
        InitWheels();
        InitAux();
        InitDc();
        InitLL();
        InitServo();

        waitForStart();

        while (opModeIsActive() && !homed) {
            homeSpinner();
            telemetry.addLine("HOMING SPINNER...");
            telemetry.update();
        }

        while (opModeIsActive()) {
            servoLogic();
            updateTelemetry();
            SetWheelsPower();
            if (!homed) { spinner.setPower(0); continue; }
            if (!outtakeMode) protectBlindSpot();
            pidSPinnerLogic();
            flywheelLogic();

            if (gamepad1.circleWasPressed()) {
                intake.setPower(-1);
                intakeMode = true;
                outtakeMode = false;
            }

            if (intakeMode) {
                updateCulori();
                colorDrivenSpinnerLogic();
            }

            if (gamepad1.squareWasPressed()) {
                outtakeMode = true;
                intakeMode = false;
                intake.setPower(0);
                outtakeTimeout.reset();
                outtakeBaseSlot = getCurrentSlot(); // latch starting slot
                integralSum = 0;
                lastError = 0;
                step1Done = false; step2Done = false; step3Done = false;
                step4Done = false; step5Done = false; step6Done = false;
                step7Done = false; step8Done = false; step9Done = false; step10Done = false;
            }

            if (outtakeMode) runOuttake();

            idle();
        }
    }
}