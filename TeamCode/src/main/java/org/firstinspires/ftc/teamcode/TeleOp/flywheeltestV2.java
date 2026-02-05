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

@TeleOp(name = "&flywheeltestV2")
public class flywheeltestV2 extends LinearOpMode {

    /* ===================== SENSOR SMOOTHING (live debug) ===================== */
    int[] last5Sensor1 = new int[5];
    int[] last5Sensor2 = new int[5];
    int[] last5Sensor3 = new int[5];

    int indexSensor1 = 0;
    int indexSensor2 = 0;
    int indexSensor3 = 0;

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

    /* ===================== OUTTAKE STEPS (legacy - not used by FSM) ===================== */
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

    double rpm = 0;
    final double[] slotPositionsIntake = {0, 0.19, 0.38};

    PinpointLocalizer pinpoint;
    Pose pose;
    double CoordX, CoordY, header;
    double Posspinner = 0;

    int ballsLoaded = 0;

    /* ===================== SPINDEXER OFFSETS / POSITIONS ===================== */
    private static final double SPINDEXER_OUTTAKE_OFFSET = -0.015;
    private static final double SPINDEXER_INTAKE_OFFSET  = -0.01;
    private static final double SPINNER_LAUNCH_POS       = 0.085; // keep same

    // When full, we park at launch pos until shooting (outtake) finishes
    private boolean launchPrepActive = false;

    /* ===================== FLYWHEEL (REV VELOCITY CONTROL) ===================== */
    static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    static double TARGET_RPM = 3055.0;
    static double TARGET_TPS = TARGET_RPM * FLYWHEEL_TICKS_PER_REV / 60.0; // ticks/sec

    static double kP_v = 15;
    static final double kI_v = 0.0;
    static final double kD_v = 0.0;
    static double kF_v = 12.8;

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
    Servo trajectoryAngleModifier;
    boolean waitingForClear = false;
    boolean spinnerMoving = false;

    static final long DETECT_DELAY_MS = 0;
    static final long SERVO_MOVE_LOCK_MS = 45;
    long servoMoveStartMs = 0;

    /* ===================== OUTTAKE FSM + RPM GATING ===================== */
    private int outtakeStep = 0;
    private long stepStartMs = 0;

    // shoot only within +/- 100 RPM, optionally stable for a short time
    private static final double RPM_TOL = 50.0;
    private static final long RPM_STABLE_MS = 80; // prevents 1-loop spikes
    private long rpmInRangeSinceMs = 0;

    private boolean rpmInRangeStable() {
        boolean inRange = (rpm >= (TARGET_RPM - RPM_TOL)) && (rpm <= (TARGET_RPM+50));
        long now = System.currentTimeMillis();

        if (!inRange) {
            rpmInRangeSinceMs = 0;
            return false;
        }
        if (rpmInRangeSinceMs == 0) rpmInRangeSinceMs = now;
        return (now - rpmInRangeSinceMs) >= RPM_STABLE_MS;
    }

    private void startStep(int newStep) {
        outtakeStep = newStep;
        stepStartMs = System.currentTimeMillis();
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

        // NOTE: leaving your original logic untouched (even though the condition looks inverted)
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
        trajectoryAngleModifier = hardwareMap.get(Servo.class, "unghituretaoy");
        ejector.setDirection(Servo.Direction.REVERSE);
        ejector.setPosition(ejectorDown);
        spinnerFar.setPosition(0);
        spinnerCLose.setPosition(0);
        trajectoryAngleModifier.setPosition(0.00);
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

    // ===================== CHASSIS ON GAMEPAD 1 =====================
    private void SetWheelsPower() {
        double left_x = gamepad1.left_stick_x;
        double left_y = -gamepad1.left_stick_y;
        double right_x = gamepad1.right_stick_x;

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

    // ULTRA-FAST intake smoothing: 3 samples, need 2
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
        if (newColorDetected) {
            lastStableIntakeColor = intakeColor;
        }

        if (newColorDetected && !colorPending && !detectionLocked) {
            colorStartTimeMs = System.currentTimeMillis();
            colorPending = true;
        }

        if (colorPending && (System.currentTimeMillis() - colorStartTimeMs >= DETECT_DELAY_MS)) {

            logicalSlots[0] = intakeColor;
            rotateLogicalSlotsRight();

            slotIntakeIndex++;
            slotIntakeIndex = slotIntakeIndex % 3;
            Posspinner = slotPositionsIntake[slotIntakeIndex];

            waitingForClear = true;
            detectionLocked = true;
            spinnerMoving = true;
            servoMoveStartMs = System.currentTimeMillis();

            colorPending = false;
        }
    }

    // When full -> park at launch position and STAY until outtake finishes
    private void autoLaunchPrepLogic() {
        if (outtakeMode) return; // during outtake we let runOuttake own Posspinner

        if (intakeMode && isSpindexerFull()) {
            launchPrepActive = true;
        }

        if (launchPrepActive) {
            Posspinner = SPINNER_LAUNCH_POS; // hard hold
        }
    }

    private void flywheelLogic() {
        flywheel.setVelocityPIDFCoefficients(kP_v, kI_v, kD_v, kF_v);
        TARGET_TPS = TARGET_RPM * FLYWHEEL_TICKS_PER_REV / 60.0;

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

        rpm = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
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
            Posspinner = 0;
            slotIntakeIndex = 0;
        }

        if (gamepad1.dpadRightWasPressed()) {
            slotIntakeIndex++;
            slotIntakeIndex = slotIntakeIndex % 3;
            Posspinner = slotPositionsIntake[slotIntakeIndex];
            rotateLogicalSlotsRight();
        }

        if (gamepad1.dpadLeftWasPressed()) {
            slotIntakeIndex--;
            if (slotIntakeIndex < 0) slotIntakeIndex = 2;
            Posspinner = slotPositionsIntake[slotIntakeIndex];
            rotateLogicalSlotsLeft();
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Flywheel Target RPM", TARGET_RPM);
        telemetry.addData("kP_v", kP_v);
        telemetry.addData("kF_v", kF_v);

        telemetry.addData("outtakeMode", outtakeMode);
        telemetry.addData("outtakeStep", outtakeStep);
        telemetry.addData("rpmInRange(±100)", (rpm >= (TARGET_RPM - RPM_TOL)) && (rpm <= (TARGET_RPM + RPM_TOL)));
        telemetry.addData("rpmStable", rpmInRangeStable());
        telemetry.addData("Posspinner(raw)", Posspinner);

        telemetry.update();
    }

    /**
     * OUTTAKE LOGIC YOU REQUESTED:
     * - ejector up (ONLY when RPM in range)
     * - ejector down (after fixed time)
     * - spinner moves to next slot
     * - repeat
     *
     * Keeps your same positions:
     *  - launch: 0.085
     *  - slot2:  0.28
     *  - slot3:  0.46
     *  - end:    0
     */
    private void runOuttake() {

        // Keep intake reversed during outtake (as you had)
        intake.setPower(-1);

        // timings (same as your old delays)
        final long INITIAL_DELAY_MS = 50;
        final long EJECTOR_UP_MS = 250;
        final long EJECTOR_DOWN_MS = 350;
        final long SPINNER_MOVE_MS = 100;

        long now = System.currentTimeMillis();
        long dt = now - stepStartMs;

        switch (outtakeStep) {

            case 0:
                // snapshot inventory once
                slots[0] = logicalSlots[0];
                slots[1] = logicalSlots[1];
                slots[2] = logicalSlots[2];

                // clear logical so intake recounts after
                logicalSlots[0] = 0;
                logicalSlots[1] = 0;
                logicalSlots[2] = 0;

                Color1 = 0; Color2 = 0; Color3 = 0;

                // start at launch position
                Posspinner = 0.085;

                rpmInRangeSinceMs = 0;
                startStep(1);
                break;

            case 1:
                // initial settle time
                if (dt >= INITIAL_DELAY_MS) startStep(2);
                break;

            case 2:
                // SHOOT #1: ejector up only when rpm is in range (stable)
                if (rpmInRangeStable()) {
                    ejector.setPosition(ejectorUp);
                    startStep(3);
                }
                break;

            case 3:
                // hold ejector up
                if (dt >= EJECTOR_UP_MS) {
                    ejector.setPosition(ejectorDown);
                    startStep(4);
                }
                break;

            case 4:
                // hold ejector down
                if (dt >= EJECTOR_DOWN_MS) {
                    // move spinner to next slot
                    Posspinner = 0.28;
                    startStep(5);
                }
                break;

            case 5:
                // wait for spinner move
                if (dt >= SPINNER_MOVE_MS) {
                    rpmInRangeSinceMs = 0; // re-arm rpm stability for next shot
                    startStep(6);
                }
                break;

            case 6:
                // SHOOT #2
                if (rpmInRangeStable()) {
                    ejector.setPosition(ejectorUp);
                    startStep(7);
                }
                break;

            case 7:
                if (dt >= EJECTOR_UP_MS) {
                    ejector.setPosition(ejectorDown);
                    startStep(8);
                }
                break;

            case 8:
                if (dt >= EJECTOR_DOWN_MS) {
                    Posspinner = 0.46;
                    startStep(9);
                }
                break;

            case 9:
                if (dt >= SPINNER_MOVE_MS) {
                    rpmInRangeSinceMs = 0;
                    startStep(10);
                }
                break;

            case 10:
                // SHOOT #3
                if (rpmInRangeStable()) {
                    ejector.setPosition(ejectorUp);
                    startStep(11);
                }
                break;

            case 11:
                if (dt >= EJECTOR_UP_MS) {
                    ejector.setPosition(ejectorDown);
                    startStep(12);
                }
                break;

            case 12:
                if (dt >= EJECTOR_DOWN_MS) {

                    // end
                    Posspinner = 0;

                    // ===== AUTO RESET TO INTAKE AFTER SHOOTING (same behavior you had) =====
                    outtakeMode = false;

                    intakeMode = true;
                    spinIntake = true;
                    intake.setPower(-1);

                    slotIntakeIndex = 0;
                    Posspinner = 0;

                    launchPrepActive = false;
                    resetIntakeGatingAndFilters();

                    ballsLoaded = 0;

                    // reset FSM
                    outtakeStep = 0;
                    stepStartMs = 0;
                    rpmInRangeSinceMs = 0;
                }
                break;
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

            // Auto park at launch when full (stays until outtake finishes)
            autoLaunchPrepLogic();

            // Apply offsets to servos
            double appliedSpinnerPos = Posspinner + SPINDEXER_INTAKE_OFFSET;
            if (outtakeMode) {
                appliedSpinnerPos = Posspinner + SPINDEXER_OUTTAKE_OFFSET;
            }
            spinnerFar.setPosition(appliedSpinnerPos);
            spinnerCLose.setPosition(appliedSpinnerPos);

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
                Posspinner = 0;
                slotIntakeIndex = 0;

                logicalSlots[0] = 0;
                logicalSlots[1] = 0;
                logicalSlots[2] = 0;

                launchPrepActive = false;
                resetIntakeGatingAndFilters();

                // reset FSM too
                outtakeStep = 0;
                stepStartMs = 0;
                rpmInRangeSinceMs = 0;
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

                // start outtake FSM cleanly
                outtakeStep = 0;
                stepStartMs = System.currentTimeMillis();
                rpmInRangeSinceMs = 0;

                // keep old vars harmless
                outtakeTimeout.reset();
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

            // tuning on gamepad2
            if (gamepad2.yWasPressed()) {
                TARGET_RPM += 5;
            }
            if (gamepad2.crossWasPressed()) {
                TARGET_RPM -= 5;
            }
            if (gamepad2.squareWasPressed()) {
                kP_v += 0.1;
            } else if (gamepad2.circleWasPressed()) {
                kP_v -= 0.1;
            } else if (gamepad2.leftBumperWasPressed()) {
                kF_v += 0.05;
            } else if (gamepad2.rightBumperWasPressed()) {
                kF_v -= 0.05;
            }

            updateTelemetry();
            idle();
        }
    }
}