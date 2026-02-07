package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TeleOpBtExperimental")
public class experimental extends LinearOpMode {

    /* ===================== SENSOR SMOOTHING (live debug) ===================== */
    int[] last5Sensor1 = new int[5];
    int[] last5Sensor2 = new int[5];
    int[] last5Sensor3 = new int[5];

    int indexSensor1 = 0;
    int indexSensor2 = 0;
    int indexSensor3 = 0;
    double offset=0;

    /* ===================== OUTTAKE SLOTS (existing) ===================== */
    int[] slots = new int[3];
    int[] totem = {2, 1, 2};

    /* ===================== LIVE COLOR READINGS (debug only) ===================== */
    int Color1 = 0;
    int Color2 = 0;
    int Color3 = 0;

    int detectedBalls = 0;
    int slotIntakeIndex = 0;
    double prev_t = 0;

    /* ===================== HARDWARE ===================== */
    ColorSensor colorsensorSLot1;
    ColorSensor colorsensorSLot2;
    ColorSensor colorsensorSLot3;

    DcMotor intake;
    DcMotorEx tureta;
    Servo ejector;

    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;

    DcMotorEx flywheel;

    Servo spinnerCLose;
    Servo spinnerFar;

    /* ===================== OUTTAKE STEPS ===================== */
    boolean step1Done = false;
    boolean turetaDisabled = false;
    boolean step2Done = false;
    boolean step3Done = false;
    boolean step4Done = false;
    boolean step5Done = false;
    boolean step6Done = false;
    boolean step7Done = false;
    boolean step8Done = false;
    boolean step9Done = false;
    boolean step10Done = false;
    boolean step11Done = false;
    boolean spinIntake = false;

    Servo trajectoryAngleModifier;
    /* ===================== LL / IMU ===================== */
    Limelight3A limelight;
    IMU imu;

    /* ===================== MODES ===================== */
    boolean intakeMode = false;
    boolean outtakeMode = false;

    private ElapsedTime spinnerTimeout = new ElapsedTime();
    private ElapsedTime outtakeTimeout = new ElapsedTime();

    final double ejectorDown = 0.214;
    final double ejectorUp = 0.03;

    final double[] slotPositionsIntake = {0, 0.19, 0.38};

    PinpointLocalizer pinpoint;
    Pose pose;
    double CoordX, CoordY, header;

    int ballsLoaded = 0;

    /* ===================== SPINDEXER OFFSETS / POSITIONS ===================== */
    // Păstrăm launch logic-ul, dar NU mai aplicăm aici offsets direct pe setPosition,
    // fiindcă folosim controller-ul cu offsets separat pe fiecare servo.
    private static final double SPINNER_LAUNCH_POS = 0.085; // launch/prep position

    // When full, we park at launch pos until outtake finishes
    private boolean launchPrepActive = false;

    /* ===================== FLYWHEEL (REV VELOCITY CONTROL) ===================== */
    static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    static final double TARGET_RPM = 2471.0;
    static final double TARGET_TPS = TARGET_RPM * FLYWHEEL_TICKS_PER_REV / 60.0;

    static final double kP_v = 12.0;
    static final double kI_v = 0.0;
    static final double kD_v = 0.0;
    static final double kF_v = 14.0;

    static final double KICK_POWER = 1.0;
    static final double KICK_TIME_S = 0.20;

    boolean flywheelOn = false;
    ElapsedTime flyKickTimer = new ElapsedTime();
    boolean kicking = false;

    /* ===================== TURRET ===================== */
    private static final double MOTOR_TICKS_PER_REV = 384.5;
    private static final double MOTOR_TO_TURRET_RATIO = 76.0 / 24.0;

    private static final double DEG_PER_TICK_TURETS =
            360.0 / (MOTOR_TICKS_PER_REV * MOTOR_TO_TURRET_RATIO);

    private static final double LEFT_LIMIT  = -110;
    private static final double RIGHT_LIMIT = 110;

    private static final double kP = 0.015;
    private static final double MAX_POWER = 0.2;

    /* ================= LOCALIZATION ================= */
    private Pose startPose;
    private double pX, pY;

    /* ================= TARGET ================= */
    private double xC = 0;
    private double yC = 144;

    /* ===================== LOGICAL INVENTORY ===================== */
    int[] logicalSlots = new int[3];

    // ULTRA-FAST intake smoothing (3 samples, need 2)
    int[] lastNIntake = new int[3];
    int idxIntake = 0;

    int lastStableIntakeColor = 0;

    boolean colorPending = false;
    long colorStartTimeMs = 0;

    boolean detectionLocked = false;
    boolean waitingForClear = false;
    boolean spinnerMoving = false;

    static final long DETECT_DELAY_MS = 0;
    static final long SERVO_MOVE_LOCK_MS = 45;
    long servoMoveStartMs = 0;

    /* =========================================================================================
       ===================== SPINNER SERVO CENTERING (deadzone/backlash fix) ====================
       ========================================================================================= */

    private static final double SPINNER_FAR_OFFSET   = -0.010;
    private static final double SPINNER_CLOSE_OFFSET = -0.010;

    private static final double SPINNER_OVERSHOOT = 0.010;
    private static final double SPINNER_OVERSHOOT_TIME_S = 0.06; // 60ms
    private static final double SPINNER_TARGET_EPS = 0.0015;

    private double spinnerTarget = 0.0;      // 0..1 (fără offset)
    private double spinnerCmd = 0.0;         // comanda curentă
    private double spinnerLastTarget = 0.0;

    private boolean spinnerInOvershoot = false;
    private double spinnerOvershootCmd = 0.0;
    private final ElapsedTime spinnerMoveTimer = new ElapsedTime();

    private void setSpinnerTarget(double target) {
        spinnerTarget = Range.clip(target, 0.0, 1.0);
    }

    private void updateSpinnerServos() {
        // retrigger overshoot doar la schimbări reale
        if (Math.abs(spinnerTarget - spinnerLastTarget) > SPINNER_TARGET_EPS) {
            spinnerLastTarget = spinnerTarget;

            double dir = Math.signum(spinnerTarget - spinnerCmd);
            if (dir == 0) dir = 1.0;

            spinnerOvershootCmd = Range.clip(spinnerTarget + dir * SPINNER_OVERSHOOT, 0.0, 1.0);
            spinnerInOvershoot = true;
            spinnerMoveTimer.reset();
        }

        if (spinnerInOvershoot) {
            spinnerCmd = spinnerOvershootCmd;

            if (spinnerMoveTimer.seconds() >= SPINNER_OVERSHOOT_TIME_S) {
                spinnerInOvershoot = false;
                spinnerCmd = spinnerTarget;
            }
        } else {
            spinnerCmd = spinnerTarget;
        }

        double farPos = Range.clip(spinnerCmd + SPINNER_FAR_OFFSET, 0.0, 1.0);
        double closePos = Range.clip(spinnerCmd + SPINNER_CLOSE_OFFSET, 0.0, 1.0);

        spinnerFar.setPosition(farPos+offset);
        spinnerCLose.setPosition(closePos+offset);
    }

    /* ========================================================================================= */

    private void initLocalization() {
        pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        startPose = new Pose(22, 127, Math.toRadians(-36));
        pinpoint.setStartPose(startPose);
    }

    private void resetLocalization() {
        Pose pose = new Pose(22, 127, Math.toRadians(-36));
        pinpoint.setPose(pose);
    }

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
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        tureta = hardwareMap.get(DcMotorEx.class, "tureta");
        tureta.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        tureta.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        tureta.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setVelocityPIDFCoefficients(kP_v, kI_v, kD_v, kF_v);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private void updateTurretAim() {
        Pose pose = pinpoint.getPose();
        pX = pose.getX();
        pY = pose.getY();

        double dx = xC - pX;
        double dy = yC - pY;

        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = Math.toDegrees(pose.getHeading());
        double currentTurretDeg = tureta.getCurrentPosition() * DEG_PER_TICK_TURETS - 180.0;

        double targetTurretDeg = normalizeAngle(fieldAngle - robotHeading);

        if (Math.abs(targetTurretDeg) < RIGHT_LIMIT || turetaDisabled) {
            tureta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            tureta.setPower(0);
            return;
        }

        currentTurretDeg = normalizeAngle(currentTurretDeg);
        double error = normalizeAngle(targetTurretDeg - currentTurretDeg);

        double power = error * kP;
        power = Range.clip(power, -MAX_POWER, MAX_POWER);
        tureta.setPower(power);
    }

    private void InitServo() {
        ejector = hardwareMap.get(Servo.class, "ejector");
        spinnerFar = hardwareMap.get(Servo.class, "SpinnerFar");
        spinnerCLose = hardwareMap.get(Servo.class, "SpinnerClose");

        ejector.setDirection(Servo.Direction.REVERSE);
        ejector.setPosition(ejectorDown);

        setSpinnerTarget(0.0);
        spinnerCmd = 0.0;
        spinnerLastTarget = 0.0;
        spinnerInOvershoot = false;
        trajectoryAngleModifier = hardwareMap.get(Servo.class, "unghituretaoy");
        trajectoryAngleModifier.setPosition(0.2749 );

        updateSpinnerServos();
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

        pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        Pose startPos = new Pose(0, 0, 0);
        pinpoint.setStartPose(startPos);
    }

    // ===================== CHASSIS ON GAMEPAD 2 =====================
    private void SetWheelsPower() {
        double left_x = gamepad2.left_stick_x;
        double left_y = -gamepad2.left_stick_y;
        double right_x = gamepad2.right_stick_x;

        double front_left_pw  = left_y + left_x + right_x;
        double back_left_pw   = left_y - left_x + right_x;
        double front_right_pw = left_y - left_x - right_x;
        double back_right_pw  = left_y + left_x - right_x;

        double max = Math.max(Math.abs(front_left_pw),
                Math.max(Math.abs(back_left_pw),
                        Math.max(Math.abs(front_right_pw), Math.abs(back_right_pw))));
        if (max > 1.0) {
            front_left_pw  /= max;
            back_left_pw   /= max;
            front_right_pw /= max;
            back_right_pw  /= max;
        }

        front_left.setPower(front_left_pw);
        back_left.setPower(back_left_pw);
        front_right.setPower(front_right_pw);
        back_right.setPower(back_right_pw);
    }

    private void offsetspinner()
    {
        if (gamepad2.dpadLeftWasPressed())offset+=0.01;
        if (gamepad2.dpadRightWasPressed())offset-=0.01;
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

    private int smekerie1(ColorSensor colorSensor) {
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

    private int CuloareFinala1(ColorSensor sensor, int[] last5, int index) {
        last5[index] = smekerie1(sensor);

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
        Color1 = CuloareFinala1(colorsensorSLot1, last5Sensor1, indexSensor1);
        indexSensor1 = (indexSensor1 + 1) % 5;

        Color2 = CuloareFinala1(colorsensorSLot2, last5Sensor2, indexSensor2);
        indexSensor2 = (indexSensor2 + 1) % 5;

        Color3 = CuloareFinala1(colorsensorSLot3, last5Sensor3, indexSensor3);
        indexSensor3 = (indexSensor3 + 1) % 5;
    }

    private void rotateLogicalSlotsRight() {
        int temp = logicalSlots[2];
        logicalSlots[2] = logicalSlots[1];
        logicalSlots[1] = logicalSlots[0];
        logicalSlots[0] = temp;
    }

    private void rotateLogicalSlotsLeft() {
        int temp = logicalSlots[0];
        logicalSlots[0] = logicalSlots[1];
        logicalSlots[1] = logicalSlots[2];
        logicalSlots[2] = temp;
    }

    private boolean isSpindexerFull() {
        return logicalSlots[0] != 0 && logicalSlots[1] != 0 && logicalSlots[2] != 0;
    }

    private void resetIntakeGatingAndFilters() {
        waitingForClear = false;
        detectionLocked = false;
        spinnerMoving = false;
        colorPending = false;
        lastStableIntakeColor = 0;

        for (int i = 0; i < lastNIntake.length; i++) lastNIntake[i] = 0;
        idxIntake = 0;
    }

    private int processIntakeSensor(ColorSensor sensor) {
        int detected = smekerie1(sensor);

        lastNIntake[idxIntake] = detected;
        idxIntake = (idxIntake + 1) % lastNIntake.length;

        int count1 = 0, count2 = 0;
        for (int v : lastNIntake) {
            if (v == 1) count1++;
            else if (v == 2) count2++;
        }

        int finalColor = 0;
        if (count1 >= 2 && count1 > count2) finalColor = 1;
        else if (count2 >= 2 && count2 > count1) finalColor = 2;

        return finalColor;
    }

    private void colorDrivenSpinnerLogicServos() {

        if (spinnerMoving) {
            if (System.currentTimeMillis() - servoMoveStartMs >= SERVO_MOVE_LOCK_MS) {
                spinnerMoving = false;
                detectionLocked = false;
                lastStableIntakeColor = 0;
            } else {
                return;
            }
        }

        if (waitingForClear) {
            int intakeColorNow = processIntakeSensor(colorsensorSLot1);
            if (intakeColorNow == 0) {
                waitingForClear = false;
                lastStableIntakeColor = 0;
            }
            return;
        }

        int intakeColor = processIntakeSensor(colorsensorSLot1);

        boolean newColorDetected = (intakeColor != 0 && lastStableIntakeColor == 0);
        if (newColorDetected) lastStableIntakeColor = intakeColor;

        if (newColorDetected && !colorPending && !detectionLocked) {
            colorStartTimeMs = System.currentTimeMillis();
            colorPending = true;
        }

        if (colorPending && (System.currentTimeMillis() - colorStartTimeMs >= DETECT_DELAY_MS)) {

            logicalSlots[0] = intakeColor;
            rotateLogicalSlotsRight();

            slotIntakeIndex++;
            slotIntakeIndex = slotIntakeIndex % 3;

            // IMPORTANT: set target (nu setPosition direct)
            setSpinnerTarget(slotPositionsIntake[slotIntakeIndex]);

            waitingForClear = true;
            detectionLocked = true;
            spinnerMoving = true;
            servoMoveStartMs = System.currentTimeMillis();

            colorPending = false;
        }
    }

    // When full -> park at launch position and STAY until outtake finishes
    private void autoLaunchPrepLogic() {
        if (outtakeMode) return;

        if (intakeMode && isSpindexerFull()) {
            launchPrepActive = true;
        }

        if (launchPrepActive) {
            setSpinnerTarget(SPINNER_LAUNCH_POS);
        }
    }

    private void flywheelLogic() {
        if (gamepad1.shareWasPressed()) {
            flywheelOn = !flywheelOn;

            if (flywheelOn) {
                kicking = true;
                flyKickTimer.reset();
            } else {
                kicking = false;
                flywheel.setPower(0);
            }
        }

        if (!flywheelOn) {
            flywheel.setPower(0);
            return;
        }

        if (kicking) {
            flywheel.setPower(KICK_POWER);
            if (flyKickTimer.seconds() >= KICK_TIME_S) {
                kicking = false;
            }
        } else {
            flywheel.setVelocity(TARGET_TPS);
        }

        double rpm = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
        telemetry.addData("Flywheel RPM", rpm);
        telemetry.addData("Flywheel mode", kicking ? "KICK" : "HOLD");
    }

    private void servoLogic() {
        if (gamepad1.optionsWasReleased()) {
            ejector.setPosition(ejectorDown);
        } else if (gamepad1.optionsWasPressed()) {
            ejector.setPosition(ejectorUp);
        }

        if (gamepad1.touchpadWasPressed()) {
            setSpinnerTarget(0);
            slotIntakeIndex = 0;
        }

        if (gamepad1.dpadRightWasPressed()) {
            slotIntakeIndex++;
            slotIntakeIndex = slotIntakeIndex % 3;
            setSpinnerTarget(slotPositionsIntake[slotIntakeIndex]);
            rotateLogicalSlotsRight();
        }

        if (gamepad1.dpadLeftWasPressed()) {
            slotIntakeIndex--;
            if (slotIntakeIndex < 0) slotIntakeIndex = 2;
            setSpinnerTarget(slotPositionsIntake[slotIntakeIndex]);
            rotateLogicalSlotsLeft();
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Live Color1", Color1);
        telemetry.addData("Live Color2", Color2);
        telemetry.addData("Live Color3", Color3);

        telemetry.addData("LogicalSlot 1", logicalSlots[0]);
        telemetry.addData("LogicalSlot 2", logicalSlots[1]);
        telemetry.addData("LogicalSlot 3", logicalSlots[2]);

        telemetry.addData("FULL", isSpindexerFull());
        telemetry.addData("launchPrepActive", launchPrepActive);

        telemetry.addData("waitingForClear", waitingForClear);
        telemetry.addData("colorPending", colorPending);
        telemetry.addData("spinnerMoving", spinnerMoving);

        telemetry.addData("spinnerTarget", spinnerTarget);
        telemetry.addData("spinnerCmd", spinnerCmd);
        telemetry.addData("spinnerOvershoot", spinnerInOvershoot);

        telemetry.addData("SpinnerFar pos", spinnerFar.getPosition());
        telemetry.addData("SpinnerClose pos", spinnerCLose.getPosition());
        telemetry.addData("slotIndex", slotIntakeIndex);

        telemetry.update();
    }

    private void runOuttake() {
        intake.setPower(-1);

        final int EJECTOR_UP_DELAY = 300;
        final int EJECTOR_DOWN_DELAY = 170;
        final int SPINNER_SLOT_CHANGE_DELAY = 300;
        final int INITIAL_DELAY = 400;

        slots[0] = logicalSlots[0];
        slots[1] = logicalSlots[1];
        slots[2] = logicalSlots[2];

        logicalSlots[0] = 0;
        logicalSlots[1] = 0;
        logicalSlots[2] = 0;

        Color1 = 0;
        Color2 = 0;
        Color3 = 0;

        double t = outtakeTimeout.milliseconds();

        // ========= PORTED OUTTAKE SPINNER VALUES from your tuned code =========
        if (t - prev_t >= 10 && !step1Done) {
            setSpinnerTarget(0.085);
            step1Done = true;
            prev_t = 10;
        }

        if (t >= prev_t + INITIAL_DELAY && !step2Done && step1Done) {
            ejector.setPosition(ejectorUp);
            step2Done = true;
            prev_t += INITIAL_DELAY;
        }

        if (t >= prev_t + EJECTOR_UP_DELAY && !step3Done && step2Done) {
            ejector.setPosition(ejectorDown);
            step3Done = true;
            prev_t += EJECTOR_UP_DELAY;
        }

        if (t >= prev_t + EJECTOR_DOWN_DELAY && !step4Done && step3Done) {
            setSpinnerTarget(0.28);
            step4Done = true;
            prev_t += EJECTOR_DOWN_DELAY;
        }

        if (t >= prev_t + SPINNER_SLOT_CHANGE_DELAY && !step5Done && step4Done) {
            ejector.setPosition(ejectorUp);
            step5Done = true;
            prev_t += SPINNER_SLOT_CHANGE_DELAY;
        }

        if (t >= prev_t + EJECTOR_UP_DELAY && !step6Done && step5Done) {
            ejector.setPosition(ejectorDown);
            step6Done = true;
            prev_t += EJECTOR_UP_DELAY;
        }

        if (t >= prev_t + EJECTOR_DOWN_DELAY && !step7Done && step6Done) {
            setSpinnerTarget(0.467);
            step7Done = true;
            prev_t += EJECTOR_DOWN_DELAY;
        }

        if (t >= prev_t + SPINNER_SLOT_CHANGE_DELAY && !step8Done && step7Done) {
            ejector.setPosition(ejectorUp);
            step8Done = true;
            prev_t += SPINNER_SLOT_CHANGE_DELAY;
        }

        if (t >= prev_t + EJECTOR_UP_DELAY && !step9Done && step8Done) {
            ejector.setPosition(ejectorDown);
            step9Done = true;
            prev_t += EJECTOR_UP_DELAY;
        }

        if (t >= prev_t + EJECTOR_DOWN_DELAY && !step10Done && step9Done) {
            setSpinnerTarget(0.0);
            step10Done = true;

            outtakeMode = false;

            // Auto return to intake
            intakeMode = true;
            spinIntake = true;
            intake.setPower(-1);

            slotIntakeIndex = 0;
            setSpinnerTarget(0.0);

            launchPrepActive = false;
            resetIntakeGatingAndFilters();

            ballsLoaded = 0;
            prev_t = 0;
        }
    }

    @Override
    public void runOpMode() {
        InitWheels();
        InitAux();
        InitDc();
        InitLL();
        InitServo();
        initLocalization();

        waitForStart();

        while (opModeIsActive()) {

            // Launch hold (full) -> sets spinnerTarget
            autoLaunchPrepLogic();

            // IMPORTANT: drive servos via controller every loop
            updateSpinnerServos();

            servoLogic();
            SetWheelsPower();
            flywheelLogic();

            pinpoint.update();
            // updateTurretAim();

            if (gamepad1.crossWasPressed() && !gamepad1.crossWasReleased()) {
                intake.setPower(1);
            } else if (gamepad1.crossWasReleased()) {
                intake.setPower(spinIntake ? -1 : 0);
            }

            if (gamepad1.circleWasPressed()) {
                outtakeMode = false;
                intakeMode = true;

                spinIntake = !spinIntake;
                intake.setPower(spinIntake ? -1 : 0);

                ballsLoaded = 0;
                setSpinnerTarget(0);
                slotIntakeIndex = 0;

                logicalSlots[0] = 0;
                logicalSlots[1] = 0;
                logicalSlots[2] = 0;

                launchPrepActive = false;
                resetIntakeGatingAndFilters();
            }

            // Only run intake detection if we are not parked at launch
            if (intakeMode && !outtakeMode && !launchPrepActive) {
                updateCulori();
                colorDrivenSpinnerLogicServos();
            } else if (intakeMode && launchPrepActive) {
                updateCulori();
            }

            if (gamepad1.psWasPressed()) {
                resetLocalization();
            }

            if (gamepad1.right_trigger > 0.8) {
                intakeMode = false;
                outtakeMode = true;
                intake.setPower(0);
                outtakeTimeout.reset();

                // outtake owns spinnerTarget now; release launch hold
                launchPrepActive = false;

                step1Done = false;
                step2Done = false;
                step3Done = false;
                step4Done = false;
                step5Done = false;
                step6Done = false;
                step7Done = false;
                step8Done = false;
                step9Done = false;
                step10Done = false;

                prev_t = 0;
            }

            if (gamepad1.yWasPressed() && !gamepad1.yWasReleased()) {
                turetaDisabled = true;
            } else if (gamepad1.yWasReleased()) {
                turetaDisabled = false;
                tureta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if (outtakeMode) {
                runOuttake();
            }

            offsetspinner();

            updateTelemetry();
            idle();
        }
    }
}