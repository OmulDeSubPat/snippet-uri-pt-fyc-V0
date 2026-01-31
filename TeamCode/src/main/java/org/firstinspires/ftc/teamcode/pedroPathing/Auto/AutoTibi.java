package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "&AutoTibichiBlueFar", group = "Autonomous")
@Configurable
public class AutoTibi extends OpMode {

    /* ===================== TELEMETRY ===================== */
    private TelemetryManager panelsTelemetry;

    /* ===================== PEDRO ===================== */
    public Follower follower;
    private int pathState = 0;
    private boolean pathStarted = false;
    private Paths paths;

    /* ===================== AUTONOMOUS STAGE ===================== */
    int autoStage = 0;
    /*
        0 = shoot preload
        1 = path1 -> path2 intake ON, path3 intake OFF
        2 = final shooting
        3 = done
     */

    /* ===================== DELAY ===================== */
    ElapsedTime autoDelay = new ElapsedTime();
    boolean waiting = false;
    final double COMMAND_DELAY = 2.5; // seconds

    private boolean delayDone() {
        if (!waiting) {
            waiting = true;
            autoDelay.reset();
            return false;
        }
        if (autoDelay.seconds() >= COMMAND_DELAY) {
            waiting = false;
            return true;
        }
        return false;
    }

    /* ===================== HARDWARE ===================== */
    DcMotor intake;
    DcMotorEx flywheel;
    Servo spinnerCLose, spinnerFar, ejector;
    ColorSensor colorsensorSLot1, colorsensorSLot2, colorsensorSLot3;
    Servo trajectoryAngleModifier;

    /* ===================== FLYWHEEL (PIDF VELOCITY) ===================== */
    static final double FLYWHEEL_TICKS_PER_REV = 28.0;

    public static double TARGET_RPM = 3800;

    public static double kP_v = 10.0;
    public static final double kI_v = 0.0;
    public static final double kD_v = 0.0;
    public static double kF_v = 13.0;

    double targetTPS;

    /* ===================== (FROM TELEOP) SENSOR SMOOTHING (debug) ===================== */
    int[] last5Sensor1 = new int[5];
    int[] last5Sensor2 = new int[5];
    int[] last5Sensor3 = new int[5];

    int indexSensor1 = 0;
    int indexSensor2 = 0;
    int indexSensor3 = 0;

    int Color1 = 0;
    int Color2 = 0;
    int Color3 = 0;

    /* ===================== (FROM TELEOP) OUTTAKE SLOTS (existing) ===================== */
    int[] slots = new int[3];
    int[] totem = {2, 1, 2};

    int detectedBalls = 0;
    int slotIntakeIndex = 0;
    double prev_t = 0;

    /* ===================== (FROM TELEOP) OUTTAKE STEPS ===================== */
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
    boolean step11Done = false;

    /* ===================== (FROM TELEOP) MODES ===================== */
    boolean intakeMode = false;
    boolean outtakeMode = false;
    boolean spinIntake = false;

    private ElapsedTime outtakeTimeout = new ElapsedTime();

    final double ejectorDown = 0.214;
    final double ejectorUp = 0.03;

    final double[] slotPositionsIntake = {0, 0.19, 0.38};
    double Posspinner = 0;

    int ballsLoaded = 0;

    /* ===================== (FROM TELEOP) SPINDEXER OFFSETS / POSITIONS ===================== */
    private static final double SPINDEXER_OUTTAKE_OFFSET = -0.015;
    private static final double SPINDEXER_INTAKE_OFFSET  = -0.01;
    private static final double SPINNER_LAUNCH_POS       = 0.085;

    private boolean launchPrepActive = false;

    /* ===================== (FROM TELEOP) LOGICAL INVENTORY ===================== */
    int[] logicalSlots = new int[3];

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

    /* ===================== INIT ===================== */
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(64.262, 8.000, Math.toRadians(90)));
        paths = new Paths(follower);

        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(kP_v, kI_v, kD_v, kF_v);

        // keep your Auto direction (you already had it)
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        spinnerCLose = hardwareMap.get(Servo.class, "SpinnerClose");
        spinnerFar = hardwareMap.get(Servo.class, "SpinnerFar");
        ejector = hardwareMap.get(Servo.class, "ejector");

        trajectoryAngleModifier = hardwareMap.get(Servo.class, "unghituretaoy");

        colorsensorSLot1 = hardwareMap.colorSensor.get("Color1");
        colorsensorSLot2 = hardwareMap.colorSensor.get("Color2");
        colorsensorSLot3 = hardwareMap.colorSensor.get("Color3");

        ejector.setDirection(Servo.Direction.REVERSE);
        ejector.setPosition(ejectorDown);

        spinnerFar.setPosition(0);
        spinnerCLose.setPosition(0);

        trajectoryAngleModifier.setPosition(0);

        autoDelay.reset();
        outtakeTimeout.reset();

        // Start state
        autoStage = 0;
        pathState = 0;
        pathStarted = false;

        intakeMode = false;
        outtakeMode = false;
        spinIntake = false;

        Posspinner = 0;
        slotIntakeIndex = 0;
        prev_t = 0;

        launchPrepActive = false;
        resetIntakeGatingAndFilters();

        logicalSlots[0] = logicalSlots[1] = logicalSlots[2] = 0;
    }

    /* ===================== LOOP ===================== */
    @Override
    public void loop() {
        follower.update();

        updateFlywheel();

        // TeleOp-like debug colors (optional, but copied)
        updateCulori();

        // Run teleop intake detection only when we are in intakeMode AND not launch-holding
        if (intakeMode && !outtakeMode && !launchPrepActive) {
            colorDrivenSpinnerLogicServos();
        }

        // Auto park at launch when full (stays until outtake finishes)
        autoLaunchPrepLogic();

        // Apply offsets like TeleOp (one place owns servo write)
        double appliedSpinnerPos = Posspinner + SPINDEXER_INTAKE_OFFSET;
        if (outtakeMode) {
            appliedSpinnerPos = Posspinner + SPINDEXER_OUTTAKE_OFFSET;
        }
        spinnerFar.setPosition(appliedSpinnerPos);
        spinnerCLose.setPosition(appliedSpinnerPos);

        // Run TeleOp outtake sequence if enabled
        if (outtakeMode) {
            runOuttake();
        }

        // ===================== AUTONOMOUS FSM =====================
        switch (autoStage) {

            // 0) Shoot preload
            case 0:
                // Start outtake once (no renames, we drive via TeleOp flags)
                if (!outtakeMode && delayDone()) {
                    intakeMode = false;
                    outtakeMode = true;
                    spinIntake = false;
                    intake.setPower(0);

                    outtakeTimeout.reset();

                    step1Done = step2Done = step3Done = step4Done = step5Done =
                            step6Done = step7Done = step8Done = step9Done = step10Done = false;
                    prev_t = 0;

                    autoStage = 1;
                }
                break;

            // 1) Path1 -> Path2 intake ON, then Path3 intake OFF
            case 1:
                // Wait until preload outtake completes (TeleOp runOuttake sets outtakeMode=false)
                if (!outtakeMode && delayDone()) {
                    // Start paths
                    pathState = 0;
                    pathStarted = false;

                    // Intake ON from Path1 through Path2 (TeleOp style)
                    intakeMode = true;
                    spinIntake = true;
                    intake.setPower(-1);

                    autoStage = 10;
                }
                break;

            // Drive + collect (Path1 + Path2 intake ON, Path3 intake OFF)
            case 10:
                autonomousPathUpdate();

                // Intake ON while in Path1 or Path2
                if (pathState == 0 || pathState == 1) {
                    intakeMode = true;
                    spinIntake = true;
                    intake.setPower(-1);
                }

                // Intake OFF at Path3 start/through finish
                if (pathState == 2 || pathState == 3) {
                    intakeMode = false;
                    spinIntake = false;
                    intake.setPower(0);
                }

                // When Path3 finishes -> final shooting
                if (pathState == 3 && delayDone()) {
                    autoStage = 2;
                }
                break;

            // 2) Final shooting
            case 2:
                if (!outtakeMode && delayDone()) {
                    intakeMode = false;
                    outtakeMode = true;
                    spinIntake = false;
                    intake.setPower(0);

                    outtakeTimeout.reset();

                    step1Done = step2Done = step3Done = step4Done = step5Done =
                            step6Done = step7Done = step8Done = step9Done = step10Done = false;
                    prev_t = 0;

                    autoStage = 3;
                }
                break;

            // 3) Done (wait outtake finish)
            case 3:
                if (!outtakeMode) {
                    intake.setPower(0);
                    autoStage = 4;
                }
                break;

            case 4:
                // done
                intake.setPower(0);
                break;
        }

        panelsTelemetry.debug("Auto Stage", autoStage);
        panelsTelemetry.debug("Path State", pathState);

        panelsTelemetry.debug("Live Color1", Color1);
        panelsTelemetry.debug("Live Color2", Color2);
        panelsTelemetry.debug("Live Color3", Color3);

        panelsTelemetry.debug("LogicalSlot 1", logicalSlots[0]);
        panelsTelemetry.debug("LogicalSlot 2", logicalSlots[1]);
        panelsTelemetry.debug("LogicalSlot 3", logicalSlots[2]);

        panelsTelemetry.debug("FULL", isSpindexerFull());
        panelsTelemetry.debug("launchPrepActive", launchPrepActive);
        panelsTelemetry.debug("outtakeMode", outtakeMode);
        panelsTelemetry.debug("intakeMode", intakeMode);

        panelsTelemetry.update(telemetry);
    }

    /* ===================== PATH FSM (Path1 -> Path2 -> Path3) ===================== */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Path1
                if (!pathStarted) {
                    follower.followPath(paths.Path1, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = 1;
                }
                break;

            case 1: // Path2
                if (!pathStarted) {
                    follower.followPath(paths.Path2, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = 2;
                }
                break;

            case 2: // Path3
                if (!pathStarted) {
                    follower.followPath(paths.Path3, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = 3;
                }
                break;

            case 3:
                break;
        }
    }

    /* ===================== FLYWHEEL ===================== */
    private void updateFlywheel() {
        flywheel.setVelocityPIDFCoefficients(kP_v, kI_v, kD_v, kF_v);

        targetTPS = TARGET_RPM * FLYWHEEL_TICKS_PER_REV / 60.0;
        flywheel.setVelocity(targetTPS);

        double currentRPM = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
        panelsTelemetry.debug("Flywheel RPM", currentRPM);
        panelsTelemetry.debug("Flywheel Target", TARGET_RPM);
    }

    /* ===================== TELEOP INTAKE/OUTTAKE IMPORT ===================== */

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
        if (outtakeMode) return;

        if (intakeMode && isSpindexerFull()) {
            launchPrepActive = true;
        }

        if (launchPrepActive) {
            Posspinner = SPINNER_LAUNCH_POS;
        }
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

        if (t - prev_t >= 10 && !step1Done) {
            Posspinner = 0.085;
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
            Posspinner = 0.28;
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
            Posspinner = 0.46;
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
            Posspinner = 0;
            step10Done = true;

            // AUTO RESET TO INTAKE AFTER SHOOTING (TeleOp behavior)
            outtakeMode = false;

            intakeMode = true;
            spinIntake = true;
            intake.setPower(-1);

            slotIntakeIndex = 0;
            Posspinner = 0;

            launchPrepActive = false;
            resetIntakeGatingAndFilters();

            ballsLoaded = 0;
            prev_t = 0;
        }
    }

    /* ===================== PATHS (3 paths) ===================== */
    public static class Paths {
        public PathChain Path1, Path2, Path3;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(64.262, 8.000), new Pose(56.327, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56.327, 60.000), new Pose(8.085, 59.658)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(8.085, 59.658), new Pose(57.935, 5.904)))
                    .setConstantHeadingInterpolation(Math.toRadians(110))
                    .build();
        }
    }
}
