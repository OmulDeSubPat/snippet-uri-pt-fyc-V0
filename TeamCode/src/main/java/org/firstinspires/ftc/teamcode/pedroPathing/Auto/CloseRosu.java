package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "&AutoRedClose", group = "Autonomous")
@Configurable
public class CloseRosu extends OpMode {

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
        This version does:
        - Stage 0: run Path0
        - Stage 1: shoot (BLOCK until shooting finishes)
        - Stage 2: run Path1
        - Stage 3: run Path2 activate intake
        - Stage 4: run Path3 takes the balls
        - Stage 5:run Path 4
        - stage 6 : shoot

     */

    /* ===================== DELAY GATE ===================== */
    ElapsedTime autoDelay = new ElapsedTime();
    boolean waiting = false;
    DcMotorEx tureta;
    final double COMMAND_DELAY = 0.0; // set to 0 for instant; change if you want a pause before each shoot

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

    // imported from TeleOp shooter
    public static double TARGET_RPM = 2500;
    public static double kP_v = 10;
    public static final double kI_v = 0.0;
    public static final double kD_v = 0.0;
    public static double kF_v = 14;

    private double targetTPS;
    private double rpm = 0.0;

    /* ===================== SENSOR SMOOTHING (debug) ===================== */
    int[] last5Sensor1 = new int[5];
    int[] last5Sensor2 = new int[5];
    int[] last5Sensor3 = new int[5];

    int indexSensor1 = 0;
    int indexSensor2 = 0;
    int indexSensor3 = 0;

    int Color1 = 0;
    int Color2 = 0;
    int Color3 = 0;

    /* ===================== OUTTAKE SLOTS ===================== */
    int[] slots = new int[3];
    int slotIntakeIndex = 0;

    /* ===================== MODES ===================== */
    boolean intakeMode = false;
    boolean outtakeMode = false;
    boolean spinIntake = false;

    final double ejectorDown = 0.214;
    final double ejectorUp = 0.03;

    final double[] slotPositionsIntake = {0, 0.19, 0.38};
    double Posspinner = 0;

    /* ===================== SPINDEXER OFFSETS / POSITIONS ===================== */
    private static final double SPINDEXER_OUTTAKE_OFFSET = -0.015;
    private static final double SPINDEXER_INTAKE_OFFSET  = -0.01;
    private static final double SPINNER_LAUNCH_POS       = 0.085;

    private boolean launchPrepActive = false;

    /* ===================== LOGICAL INVENTORY ===================== */
    int[] logicalSlots = new int[3];

    // intake detect internal
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

    /* =======================================================================================
       SHOOTING MECHANISM (PID + RPM GATE + FSM) IMPORTED FROM TeleOp
       ======================================================================================= */

    private int outtakeStep = 0;
    private long stepStartMs = 0;

    private static final double RPM_TOL = 200.0;
    private static final long RPM_STABLE_MS = 80;
    private long rpmInRangeSinceMs = 0;
    private Pose robotPose;


    private boolean rpmInRangeStable() {
        // exactly your TeleOp asymmetric gate: [TARGET-100, TARGET+20]
        boolean inRange = (rpm >= (TARGET_RPM - RPM_TOL)) && (rpm <= (TARGET_RPM + 200.0));
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

    private static final long OUTTAKE_INITIAL_DELAY_MS = 50;
    private static final long OUTTAKE_EJECTOR_UP_MS     = 250;
    private static final long OUTTAKE_EJECTOR_DOWN_MS   = 350;
    private static final long OUTTAKE_SPINNER_MOVE_MS   = 100;

    // latch so shoot stages BLOCK until outtakeMode finishes
    private boolean shootStageStarted = false;

    /* ===================== INIT ===================== */
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(121, 125, Math.toRadians(-145)));

        paths = new Paths(follower);

        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(kP_v, kI_v, kD_v, kF_v);
        tureta = hardwareMap.get(DcMotorEx.class,"tureta");
        tureta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tureta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // you had REVERSE in your code
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        spinnerCLose = hardwareMap.get(Servo.class, "SpinnerClose");
        spinnerFar   = hardwareMap.get(Servo.class, "SpinnerFar");
        ejector      = hardwareMap.get(Servo.class, "ejector");

        trajectoryAngleModifier = hardwareMap.get(Servo.class, "unghituretaoy");
        trajectoryAngleModifier.setPosition(0);

        colorsensorSLot1 = hardwareMap.colorSensor.get("Color1");
        colorsensorSLot2 = hardwareMap.colorSensor.get("Color2");
        colorsensorSLot3 = hardwareMap.colorSensor.get("Color3");

        ejector.setDirection(Servo.Direction.REVERSE);
        ejector.setPosition(ejectorDown);

        spinnerFar.setPosition(0);
        spinnerCLose.setPosition(0);

        autoDelay.reset();

        // Start state
        autoStage = 0;
        pathState = 0;
        pathStarted = false;

        intakeMode = false;
        outtakeMode = false;
        spinIntake = false;

        Posspinner = 0;
        slotIntakeIndex = 0;

        launchPrepActive = false;
        resetIntakeGatingAndFilters();

        logicalSlots[0] = logicalSlots[1] = logicalSlots[2] = 0;

        // shooter fsm reset
        outtakeStep = 0;
        stepStartMs = 0;
        rpmInRangeSinceMs = 0;

        shootStageStarted = false;
        waiting = false;
    }

    /* ===================== LOOP ===================== */
    @Override
    public void loop() {
        follower.update();

        updateFlywheel();
        updateCulori();
        robotPose = follower.getPose();


        // intake detection only while intaking and not shooting and not launch holding
        if (intakeMode && !outtakeMode && !launchPrepActive) {
            colorDrivenSpinnerLogicServos();
        }

        // auto park at launch when full
        autoLaunchPrepLogic();

        // apply offsets to spindexer servos
        double appliedSpinnerPos = Posspinner + SPINDEXER_INTAKE_OFFSET;
        if (outtakeMode) {
            appliedSpinnerPos = Posspinner + SPINDEXER_OUTTAKE_OFFSET;
        }
        spinnerFar.setPosition(appliedSpinnerPos);
        spinnerCLose.setPosition(appliedSpinnerPos);

        // ===================== AUTONOMOUS FSM =====================
        // ===================== AUTONOMOUS FSM =====================
        switch (autoStage) {

            // Stage 0: run Path0
            case 0:
                if (!pathStarted) {
                    follower.followPath(paths.Path0, 0.7, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    waiting = false;
                    shootStageStarted = false;
                    autoStage = 1;
                }
                break;

            // Stage 1: SHOOT after Path0 (BLOCK until finished)
            case 1:
                if (!shootStageStarted) {
                    if (delayDone()) {
                        startOuttake();
                        shootStageStarted = true;
                    }
                } else {
                    if (!outtakeMode) {
                        shootStageStarted = false;
                        waiting = false;
                        autoStage = 2;
                    }
                }
                break;

            // Stage 2: run Path1
            case 2:
                if (!pathStarted) {
                    follower.followPath(paths.Path1, 1, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    autoStage = 3;
                }
                break;

            // Stage 3: run Path2 (intake ON)
            case 3:
                intakeMode = true;
                spinIntake = true;
                intake.setPower(1);

                if (!pathStarted) {
                    follower.followPath(paths.Path2, 0.4, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    autoStage = 4;
                }
                break;

            // Stage 4: run Path3 (intake OFF, take the balls)
            case 4:
                intakeMode = false;
                spinIntake = false;
                intake.setPower(0);

                if (!pathStarted) {
                    follower.followPath(paths.Path3, 1, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    waiting = false;
                    shootStageStarted = false;
                    autoStage = 5;
                }
                break;

            // Stage 5: run Path4
// Stage 5: run last path (same as Path3)
            case 5:
                if (!pathStarted) {
                    follower.followPath(paths.Path3, true); // use Path3
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    autoStage = 6; // move to shooting stage
                }
                break;



            // Stage 6: SHOOT after Path4
            case 6:
                if (!shootStageStarted) {
                    if (delayDone()) {
                        startOuttake();
                        shootStageStarted = true;
                    }
                } else {
                    if (!outtakeMode) {
                        shootStageStarted = false;
                        waiting = false;
                        autoStage = 7; // DONE
                    }
                }
                break;

            // Stage 7: DONE
            // Stage 7: go to 55,117 (intake ON)
            case 7:
                intakeMode = true;
                spinIntake = true;
                intake.setPower(1);

                if (!pathStarted) {
                    follower.followPath(paths.Path5, 1, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    autoStage = 8;
                }
                break;

// Stage 8: go to 15,117 (slow)
            case 8:
                if (!pathStarted) {
                    follower.followPath(paths.Path6, 0.4, true); // slow power
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    autoStage = 9;
                }
                break;

// Stage 9: go to 55,93 and shoot
            case 9:
                intakeMode = false;
                spinIntake = false;
                intake.setPower(0);

                if (!pathStarted) {
                    follower.followPath(paths.Path7, 1, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    autoStage = 10; // shoot stage
                }
                break;

// Stage 10: SHOOT
            case 10:
                if (!shootStageStarted) {
                    if (delayDone()) {
                        startOuttake();
                        shootStageStarted = true;
                    }
                } else {
                    if (!outtakeMode) {
                        shootStageStarted = false;
                        waiting = false;
                        autoStage = 11; // DONE
                    }
                }
                break;

// Stage 11: DONE
// Stage 11: go to FINAL park (45,86)
            case 11:
                if (!pathStarted) {
                    follower.followPath(paths.Path8, 0.7, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    autoStage = 12;
                }
                break;

            // Stage 12: FINAL DONE
            case 12:
                intake.setPower(0);
                tureta.setTargetPosition(545);
                tureta.setPower(0.1);
                tureta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;


        }


        // run shooter FSM after stage logic
        if (outtakeMode) {
            runOuttake();
        }

        // ===================== TELEMETRY =====================
        panelsTelemetry.debug("Robot X", robotPose.getX());
        panelsTelemetry.debug("Robot Y", robotPose.getY());
        panelsTelemetry.debug("Robot Heading (deg)", Math.toDegrees(robotPose.getHeading()));



        panelsTelemetry.debug("Auto Stage", autoStage);
        panelsTelemetry.debug("Path Started", pathStarted);
        panelsTelemetry.debug("Follower Busy", follower.isBusy());

        panelsTelemetry.debug("Flywheel RPM", rpm);
        panelsTelemetry.debug("Flywheel Target", TARGET_RPM);
        panelsTelemetry.debug("rpmStable", rpmInRangeStable());
        panelsTelemetry.debug("tureta ticks", tureta.getCurrentPosition());


        panelsTelemetry.update(telemetry);
    }

    /* ===================== START OUTTAKE ===================== */
    private void startOuttake() {
        intakeMode = false;
        outtakeMode = true;
        spinIntake = false;

        // feeding during shooting (with REVERSE direction, power(1) is "reverse")
        intake.setPower(1);

        // prevent launch-hold overwriting during sequence
        launchPrepActive = false;

        // reset shooter FSM
        outtakeStep = 0;
        stepStartMs = System.currentTimeMillis();
        rpmInRangeSinceMs = 0;
    }

    /* ===================== FLYWHEEL ===================== */
    private void updateFlywheel() {
        flywheel.setVelocityPIDFCoefficients(kP_v, kI_v, kD_v, kF_v);

        targetTPS = TARGET_RPM * FLYWHEEL_TICKS_PER_REV / 60.0;
        flywheel.setVelocity(targetTPS);

        rpm = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    /* ===================== OUTTAKE SEQUENCE (TeleOp shooter FSM) ===================== */
    private void runOuttake() {
        // keep feeding while shooting
        intake.setPower(1);

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

                Color1 = 0;
                Color2 = 0;
                Color3 = 0;

                // start at launch position
                Posspinner = 0.085;

                rpmInRangeSinceMs = 0;
                startStep(1);
                break;

            case 1:
                if (dt >= OUTTAKE_INITIAL_DELAY_MS) startStep(2);
                break;

            case 2:
                // SHOOT #1 only when rpm stable in range
                if (rpmInRangeStable()) {
                    ejector.setPosition(ejectorUp);
                    startStep(3);
                }
                break;

            case 3:
                if (dt >= OUTTAKE_EJECTOR_UP_MS) {
                    ejector.setPosition(ejectorDown);
                    startStep(4);
                }
                break;

            case 4:
                if (dt >= OUTTAKE_EJECTOR_DOWN_MS) {
                    Posspinner = 0.28;
                    startStep(5);
                }
                break;

            case 5:
                if (dt >= OUTTAKE_SPINNER_MOVE_MS) {
                    rpmInRangeSinceMs = 0;
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
                if (dt >= OUTTAKE_EJECTOR_UP_MS) {
                    ejector.setPosition(ejectorDown);
                    startStep(8);
                }
                break;

            case 8:
                if (dt >= OUTTAKE_EJECTOR_DOWN_MS) {
                    Posspinner = 0.46;
                    startStep(9);
                }
                break;

            case 9:
                if (dt >= OUTTAKE_SPINNER_MOVE_MS) {
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
                if (dt >= OUTTAKE_EJECTOR_UP_MS) {
                    ejector.setPosition(ejectorDown);
                    startStep(12);
                }
                break;

            case 12:
                if (dt >= OUTTAKE_EJECTOR_DOWN_MS) {

                    // end
                    Posspinner = 0;

                    outtakeMode = false;

                    intakeMode = false;
                    spinIntake = false;
                    intake.setPower(0);

                    slotIntakeIndex = 0;
                    Posspinner = 0;

                    launchPrepActive = false;
                    resetIntakeGatingAndFilters();

                    // reset shooter FSM
                    outtakeStep = 0;
                    stepStartMs = 0;
                    rpmInRangeSinceMs = 0;
                }
                break;
        }
    }

    /* ===================== COLOR DETECTION ===================== */
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
        if (outtakeMode) return;

        if (intakeMode && isSpindexerFull()) {
            launchPrepActive = true;
        }
        if (launchPrepActive) {
            Posspinner = SPINNER_LAUNCH_POS;
        }
    }

    /* ===================== PATHS ===================== */
    /* ===================== PATHS ===================== */

    public static class Paths {
        public PathChain Path0;
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;

        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;

        public Paths(Follower follower) {

            // ================= BLUE → RED MIRROR =================
            // Mirror rule:
            //   X' = 144 - X
            //   Y' = Y
            //   Heading' = -Heading

            Path0 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(121, 125),   // mirrored start
                            new Pose(159, 161)))    // mirror of (58,89)
                    .setConstantHeadingInterpolation(Math.toRadians(-145))
                    .build();

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(159, 161),
                            new Pose(159, 162)))
                    .setLinearHeadingInterpolation(Math.toRadians(-145), Math.toRadians(-180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(159, 162),///done
                            new Pose(124, 161)))
                    .setConstantHeadingInterpolation(Math.toRadians(-180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(124, 161),//done
                            new Pose(159, 161)))
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-145))
                    .build();

            // ======== Extra cycles ========

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(159, 161),
                            new Pose(163, 184)))
                    .setLinearHeadingInterpolation(Math.toRadians(-145), Math.toRadians(-180))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(163, 184),
                            new Pose(123, 184)))
                    .setConstantHeadingInterpolation(Math.toRadians(-180))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(123, 184),//done
                            new Pose(159, 161)))
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-145))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(159, 161),
                            new Pose(155, 180)))
                    .setLinearHeadingInterpolation(Math.toRadians(-145), Math.toRadians(-145))
                    .build();
        }
    }



}
//155 171