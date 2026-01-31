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

@Autonomous(name = "&testeazaastaboule", group = "Autonomous")
@Configurable
public class autotent extends OpMode {
    // ===== TELEOP-GRADE COLOR SYSTEM =====
    int[] last5Sensor1 = new int[5];
    int idx1 = 0;
    double stepDeadline = 0;


    int[] logicalSlots = new int[3];   // what is actually in the spinner
    int lastStableIntakeColor = 0;
    boolean spinnerMoving = false;
    boolean detectionLocked = false;
    boolean colorPending = false;
    boolean waitingForClear = false;
    long colorStartTimeMs = 0;
    long servoMoveStartMs = 0;

    static final long DETECT_DELAY_MS = 0;
    static final long SERVO_MOVE_LOCK_MS = 45;

    int slotIntakeIndex = 0;
    final double[] slotPositionsIntake = {0, 0.19, 0.38, 0.085};
    double Posspinner = 0;


    /* ================= TELEMETRY ================= */
    private TelemetryManager panelsTelemetry;

    /* ================= PEDRO ================= */
    public Follower follower;
    private int pathState = 0;
    private boolean pathStarted = false;
    private Paths paths;

    /* ================= AUTONOMOUS STAGE ================= */
    int autoStage = 0;
    double stepStartTime;

    ElapsedTime outtakeTimer = new ElapsedTime();
    double stepStart = 0;

    /*
    0 = initial shooting
    1 = drive + intake
    2 = final shooting
    3 = done
    */

    /* ================= DELAY ================= */
    ElapsedTime autoDelay = new ElapsedTime();
    boolean waiting = false;
    final double COMMAND_DELAY = 2.5; // seconds

    private boolean delayDone() {
        if(!waiting){
            waiting = true;
            autoDelay.reset();
            return false;
        }
        if(autoDelay.seconds() >= COMMAND_DELAY){
            waiting = false;
            return true;
        }
        return false;
    }

    /* ================= HARDWARE ================= */
    DcMotor intake;
    DcMotorEx flywheel;
    Servo spinnerClose, spinnerFar, ejector;
    ColorSensor slot1, slot2, slot3;
    Servo trajectoryAngleModifier;

    /* ================= FLYWHEEL ================= */
    /* ================= FLYWHEEL (PIDF VELOCITY) ================= */
    static final double FLYWHEEL_TICKS_PER_REV = 28.0;

    public static double TARGET_RPM = 3800;

    public static double kP_v = 10.0;
    public static final double kI_v = 0.0;
    public static final double kD_v = 0.0;
    public static double kF_v = 13.0;

    double targetTPS;


    /* ================= SPINNER ================= */
    final double[] slotPositions = {0, 0.19, 0.38, 0.085};
    int slotIndex = 0;
    int lastC1 = 0;

    /* ================= COLOR ================= */
    int[] last1 = new int[5], last2 = new int[5], last3 = new int[5];
    int i1=0, i2=0, i3=0;
    int c1, c2, c3;

    /* ================= OUTTAKE ================= */
    ElapsedTime outtakeTimeout = new ElapsedTime();
    boolean outtaking = false;

    final double ejectUp = 0.0;//0.02
    final double ejectDown = 0.135;//0.18

    // ms-based timing (same as TeleOp)
    final int EJECTOR_UP_DELAY = 700;
    final int EJECTOR_DOWN_DELAY = 800;
    final int SPINNER_SLOT_CHANGE_DELAY = 500;
    final int INITIAL_DELAY = 600;

    double prev_t = 0;

    boolean step1Done, step2Done, step3Done, step4Done, step5Done,
            step6Done, step7Done, step8Done, step9Done, step10Done;


    /* ================= INIT ================= */
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

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        spinnerClose = hardwareMap.get(Servo.class, "SpinnerClose");
        spinnerFar = hardwareMap.get(Servo.class, "SpinnerFar");
        ejector = hardwareMap.get(Servo.class, "ejector");

        trajectoryAngleModifier = hardwareMap.get(Servo.class,"unghituretaoy");
        slot1 = hardwareMap.colorSensor.get("Color1");
        slot2 = hardwareMap.colorSensor.get("Color2");
        slot3 = hardwareMap.colorSensor.get("Color3");

        ejector.setDirection(Servo.Direction.REVERSE);
        ejector.setPosition(ejectDown);

        autoDelay.reset();
        outtakeTimeout.reset();
        spinnerFar.setPosition(0);
        spinnerClose.setPosition(0);

        trajectoryAngleModifier.setPosition(0);
    }

    private double getHue(int r, int g, int b) {
        int max = Math.max(r, Math.max(g, b));
        int min = Math.min(r, Math.min(g, b));
        if (max == min) return 0;
        double c = max - min;
        double h;
        if (max == r) h = (g - b) / c;
        else if (max == g) h = (b - r) / c + 2;
        else h = (r - g) / c + 4;
        h *= 60;
        if (h < 0) h += 360;
        return h;
    }

    private int smekerie1(ColorSensor s) {
        int r = s.red(), g = s.green(), b = s.blue(), a = s.alpha();
        double h = getHue(r, g, b);
        if (a < 100 && (h == 150 || h == 144)) return 0;
        if ((h > 215) || (a < 100 && (h == 160 || h == 180))) return 2;
        if (h > 135 && h < 160 && a > 100) return 1;
        if ((h == 140 || h == 145) && a == 43) return 0;
        if (h > 135 && h < 160 && a > 60) return 1;
        if ((h == 210 || h == 220 || h == 225 || h == 200) && a < 100) return 2;
        return 0;
    }

    // 3-sample ultra fast filter
    private int processIntakeSensor(ColorSensor sensor) {
        int v = smekerie1(sensor);
        last5Sensor1[idx1] = v;
        idx1 = (idx1 + 1) % last5Sensor1.length;

        int c1 = 0, c2 = 0;
        for (int x : last5Sensor1) {
            if (x == 1) c1++;
            if (x == 2) c2++;
        }
        if (c1 >= 2 && c1 > c2) return 1;
        if (c2 >= 2 && c2 > c1) return 2;
        return 0;
    }

    private void rotateLogicalSlotsRight() {
        int t = logicalSlots[2];
        logicalSlots[2] = logicalSlots[1];
        logicalSlots[1] = logicalSlots[0];
        logicalSlots[0] = t;
    }


    /* ================= LOOP ================= */
    @Override
    public void loop() {
        follower.update();

        updateFlywheel();
        updateColors();
        if(autoStage == 1) {   // only while intaking
            colorDrivenSpinnerLogic();
        }

        runOuttake();

        switch(autoStage){

            // 🔥 Shoot preload
            case 0:
                if(delayDone()){
                    startOuttake();
                    autoStage = 10;
                }
                break;


            case 10:
                if(!outtaking && delayDone()){
                    intake.setPower(1);
                    autoStage = 1;
                }
                break;

            // 🚗 Drive + collect
            case 1:
                autonomousPathUpdate();
                if(pathState == 9 && delayDone()){
                    intake.setPower(0);
                    autoStage = 2;
                }
                break;

            // 🎯 Final shooting
            case 2:
                if(delayDone()){
                    startOuttake();
                    autoStage = 3;
                }
                break;

            case 3:
                // Done
                break;
        }

        panelsTelemetry.debug("Auto Stage", autoStage);
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.update(telemetry);
    }
    private void startOuttake(){
        outtaking = true;
        outtakeTimeout.reset();

        stepDeadline = 100;   // first event at t = 100ms

        step1Done = step2Done = step3Done = step4Done = step5Done =
                step6Done = step7Done = step8Done = step9Done = step10Done = false;
    }




    /* ================= PATH FSM ================= */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!pathStarted){ follower.followPath(paths.Path1, true); pathStarted = true; }
                if(!follower.isBusy()){ pathStarted = false; pathState = 2; }
                break;
            case 2:
                if(!pathStarted){ follower.followPath(paths.Path2, true); pathStarted = true; }
                if(!follower.isBusy()){ pathStarted = false; pathState = 4; }
                break;
            case 4:
                if(!pathStarted){ follower.followPath(paths.Path3, true); pathStarted = true; }
                if(!follower.isBusy()){ pathStarted = false; pathState = 6; }
                break;
            case 6:
                if(!pathStarted){ follower.followPath(paths.Path4, true); pathStarted = true; }
                if(!follower.isBusy()){ pathStarted = false; pathState = 8; }
                break;
            case 8:
                if(!pathStarted){ follower.followPath(paths.Path5, true); pathStarted = true; }
                if(!follower.isBusy()){ pathStarted = false; pathState = 9; }
                break;
            case 9:
                break;
        }
    }

    /* ================= MECHANISMS ================= */
    private void updateFlywheel() {
        // Allow live tuning from dashboard
        flywheel.setVelocityPIDFCoefficients(kP_v, kI_v, kD_v, kF_v);

        targetTPS = TARGET_RPM * FLYWHEEL_TICKS_PER_REV / 60.0;
        flywheel.setVelocity(targetTPS);

        double currentRPM = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;

        panelsTelemetry.debug("Flywheel RPM", currentRPM);
        panelsTelemetry.debug("Flywheel Target", TARGET_RPM);
    }


    private void updateColors() {
        c1 = smooth(slot1, last1, i1++ % 5);
        c2 = smooth(slot2, last2, i2++ % 5);
        c3 = smooth(slot3, last3, i3++ % 5);
    }

    private int smooth(ColorSensor s, int[] buf, int idx) {
        buf[idx] = rawColor(s);
        int a = 0, b = 0;
        for(int v: buf){ if(v==1) a++; if(v==2) b++; }
        if(a>=3) return 1;
        if(b>=3) return 2;
        return 0;
    }

    private int rawColor(ColorSensor s){
        int r = s.red(), g = s.green(), b = s.blue();
        if(b > r+20) return 2;
        if(r > b+20) return 1;
        return 0;
    }

    private void colorDrivenSpinnerLogic() {

        // Servo still moving
        if (spinnerMoving) {
            if (System.currentTimeMillis() - servoMoveStartMs >= SERVO_MOVE_LOCK_MS) {
                spinnerMoving = false;
                detectionLocked = false;
                lastStableIntakeColor = 0;
            } else return;
        }

        // Wait until ball clears sensor
        if (waitingForClear) {
            if (processIntakeSensor(slot1) == 0) {
                waitingForClear = false;
                lastStableIntakeColor = 0;
            }
            return;
        }

        int intakeColor = processIntakeSensor(slot1);

        boolean newBall = (intakeColor != 0 && lastStableIntakeColor == 0);
        if (newBall) lastStableIntakeColor = intakeColor;

        if (newBall && !colorPending && !detectionLocked) {
            colorStartTimeMs = System.currentTimeMillis();
            colorPending = true;
        }

        if (colorPending && System.currentTimeMillis() - colorStartTimeMs >= DETECT_DELAY_MS) {

            logicalSlots[0] = intakeColor;
            rotateLogicalSlotsRight();

            slotIntakeIndex = (slotIntakeIndex + 1) % 4;
            Posspinner = slotPositionsIntake[slotIntakeIndex];

            spinnerClose.setPosition(Posspinner);
            spinnerFar.setPosition(Posspinner);

            waitingForClear = true;
            detectionLocked = true;
            spinnerMoving = true;
            servoMoveStartMs = System.currentTimeMillis();
            colorPending = false;
        }
    }


    /* ================= OUTTAKE ================= */
    private void runOuttake() {
        if (!outtaking) return;

        double t = outtakeTimeout.milliseconds();

        if (!step1Done && t >= stepDeadline) {
            spinnerClose.setPosition(0.084);
            spinnerFar.setPosition(0.084);
            step1Done = true;
            stepDeadline += INITIAL_DELAY;
        }

        if (step1Done && !step2Done && t >= stepDeadline) {
            ejector.setPosition(ejectUp);
            step2Done = true;
            stepDeadline += EJECTOR_UP_DELAY;
        }

        if (step2Done && !step3Done && t >= stepDeadline) {
            ejector.setPosition(ejectDown);
            step3Done = true;
            stepDeadline += EJECTOR_DOWN_DELAY;
        }

        if (step3Done && !step4Done && t >= stepDeadline) {
            spinnerClose.setPosition(0.27);
            spinnerFar.setPosition(0.27);
            step4Done = true;
            stepDeadline += SPINNER_SLOT_CHANGE_DELAY;
        }

        if (step4Done && !step5Done && t >= stepDeadline) {
            ejector.setPosition(ejectUp);
            step5Done = true;
            stepDeadline += EJECTOR_UP_DELAY;
        }

        if (step5Done && !step6Done && t >= stepDeadline) {
            ejector.setPosition(ejectDown);
            step6Done = true;
            stepDeadline += EJECTOR_DOWN_DELAY;
        }

        if (step6Done && !step7Done && t >= stepDeadline) {
            spinnerClose.setPosition(0.45);
            spinnerFar.setPosition(0.45);
            step7Done = true;
            stepDeadline += SPINNER_SLOT_CHANGE_DELAY;
        }

        if (step7Done && !step8Done && t >= stepDeadline) {
            ejector.setPosition(ejectUp);
            step8Done = true;
            stepDeadline += EJECTOR_UP_DELAY;
        }

        if (step8Done && !step9Done && t >= stepDeadline) {
            ejector.setPosition(ejectDown);
            step9Done = true;
            stepDeadline += EJECTOR_DOWN_DELAY;
        }

        if (step9Done && !step10Done && t >= stepDeadline) {
            spinnerClose.setPosition(0);
            spinnerFar.setPosition(0);
            intake.setPower(0);
            outtaking = false;
            step10Done = true;
        }
    }





    /* ================= PATHS ================= */
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5;

        public Paths(Follower follower){
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(64.262, 8.000), new Pose(56.327, 60.00)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56.327, 60.000), new Pose(8.085, 59.658)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(8.085, 59.658), new Pose(57.935, 5.904)))
                    .setConstantHeadingInterpolation(110)
                    .build();

          /*  Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(28.623, 60.508), new Pose(10.787, 60.148)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(10.787, 60.148), new Pose(64.623, 8.443)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();*/
        }
    }
}
