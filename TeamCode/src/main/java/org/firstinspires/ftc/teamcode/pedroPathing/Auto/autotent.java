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

@Autonomous(name = "FullAutoUpdated", group = "Autonomous")
@Configurable
public class autotent extends OpMode {
    // ===== TELEOP-GRADE COLOR SYSTEM =====
    int[] last5Sensor1 = new int[5];
    int idx1 = 0;

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
    final double COMMAND_DELAY = 2; // seconds

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

    /* ================= FLYWHEEL ================= */
    final double TICKS_PER_REV = 28;
    final double TARGET_RPM = 4000;
    final double HIGH_PWR = 0.8;
    final double LOW_PWR = 0.55;
    final double RPM_TOL = 30;

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
    int outtakeStep = 0;
    boolean outtaking = false;
    final double ejectUp = 0.02;
    final double ejectDown = 0.19;
    double prev_t_outtake = 0.0;

    /* ================= INIT ================= */
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(64.262, 8.000, Math.toRadians(90)));
        paths = new Paths(follower);

        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        spinnerClose = hardwareMap.get(Servo.class, "SpinnerClose");
        spinnerFar = hardwareMap.get(Servo.class, "SpinnerFar");
        ejector = hardwareMap.get(Servo.class, "ejector");

        slot1 = hardwareMap.colorSensor.get("Color1");
        slot2 = hardwareMap.colorSensor.get("Color2");
        slot3 = hardwareMap.colorSensor.get("Color3");

        ejector.setDirection(Servo.Direction.REVERSE);
        ejector.setPosition(ejectDown);

        autoDelay.reset();
        outtakeTimeout.reset();
        spinnerFar.setPosition(0);
        spinnerClose.setPosition(0);
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
        double rpm = flywheel.getVelocity() / TICKS_PER_REV * 60;
        if(rpm < TARGET_RPM - RPM_TOL) flywheel.setPower(HIGH_PWR);
        else flywheel.setPower(LOW_PWR);
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
    private void startOuttake(){
        outtaking = true;
        outtakeStep = 0;
        outtakeTimer.reset();
        stepStart = 0;
    }

    private void runOuttake() {
        if(!outtaking) return;

        double t = outtakeTimer.seconds();

        switch(outtakeStep) {
            case 0:
                spinnerClose.setPosition(0.065);
                spinnerFar.setPosition(0.065);
                stepStart = t;
                outtakeStep++;
                break;

            case 1:
                if(t - stepStart > 1){
                    ejector.setPosition(ejectUp);
                    stepStart = t;
                    outtakeStep++;
                }
                break;

            case 2:
                if(t - stepStart > 1){
                    ejector.setPosition(ejectDown);
                    stepStart = t;
                    outtakeStep++;
                }
                break;

            case 3:
                if(t - stepStart > 1){
                    spinnerClose.setPosition(0.26);
                    spinnerFar.setPosition(0.26);
                    stepStart = t;
                    outtakeStep++;
                }
                break;

            case 4:
                if(t - stepStart > 1){
                    ejector.setPosition(ejectUp);
                    stepStart = t;
                    outtakeStep++;
                }
                break;

            case 5:
                if(t - stepStart > 1){
                    ejector.setPosition(ejectDown);
                    stepStart = t;
                    outtakeStep++;
                }
                break;

            case 6:
                if(t - stepStart > 1){
                    spinnerClose.setPosition(0.44);
                    spinnerFar.setPosition(0.44);
                    stepStart = t;
                    outtakeStep++;
                }
                break;

            case 7:
                if(t - stepStart > 1){
                    ejector.setPosition(ejectUp);
                    stepStart = t;
                    outtakeStep++;
                }
                break;

            case 8:
                if(t - stepStart > 1){
                    ejector.setPosition(ejectDown);
                    stepStart = t;
                    outtakeStep++;
                }
                break;

            case 9:
                if(t - stepStart > 1){
                    spinnerClose.setPosition(0);
                    spinnerFar.setPosition(0);
                    outtaking = false;
                    outtakeStep = 0;
                }
                break;
        }
    }



    /* ================= PATHS ================= */
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5;

        public Paths(Follower follower){
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(64.262, 8.000), new Pose(60.000, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 60.000), new Pose(36.492, 60.590)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(36.492, 60.590), new Pose(28.623, 60.508)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(28.623, 60.508), new Pose(10.787, 60.148)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(10.787, 60.148), new Pose(64.623, 8.443)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();
        }
    }
}
