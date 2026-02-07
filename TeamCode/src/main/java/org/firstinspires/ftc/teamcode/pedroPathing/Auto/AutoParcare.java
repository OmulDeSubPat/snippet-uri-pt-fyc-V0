package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(
        name = "&AutoStefan_Parcare",
        group = "Auto"
)
public class AutoParcare extends OpMode {

    /* ===================== PEDRO ===================== */
    private Follower follower;
    private Timer pathTimer, actionTimer, flywheelTimer;
    private int state = 0;

    /* ===================== STATES ===================== */
    private static final int START_FLYWHEEL   = 0;
    private static final int DRIVE_BACK       = 1;
    private static final int WAIT_SPINUP      = 2;   // wait until RPM reached OR 4s max
    private static final int WAIT_EXTRA_1S    = 3;   // always wait 1s after ready
    private static final int SHOOT_3          = 4;
    private static final int DONE             = -1;

    /* ===================== POSES ===================== */
    // Blue version start (your provided)
    private final Pose startPose = new Pose(22, 127, Math.toRadians(-36));
    private static final double BACK_DISTANCE = 22.0;
    private Pose backPose;

    /* ===================== PATHS ===================== */
    private Path backPath;

    /* ===================== HARDWARE ===================== */
    private DcMotor intake;
    private DcMotorEx flywheel;
    private Servo ejector, spinnerFar, spinnerClose;

    /* ===================== SERVO CONSTANTS ===================== */
    private static final double EJECTOR_DOWN = 0.285;
    private static final double EJECTOR_UP   = 0.005;

    private static final double SPINNER_0   = 0.0;
    private static final double SPINNER_PRE = 0.085;
    private static final double SPINNER_1   = 0.28;
    private static final double SPINNER_2   = 0.46;

    /* ===================== FLYWHEEL ===================== */
    private static final double TICKS_PER_REV = 28.0;
    private static final double TARGET_RPM = 2000.0;
    private static final double TARGET_TPS = TARGET_RPM * TICKS_PER_REV / 60.0;

    // REV velocity PIDF (your reference)
    private static final double kP = 12.0, kI = 0.0, kD = 0.0, kF = 14.0;

    // Fast spin-up kick
    private static final double KICK_POWER  = 1.0;
    private static final double KICK_TIME_S = 0.20;

    // Spinup gating
    private static final double RPM_READY_RATIO = 0.95; // 95% of target
    private static final double MAX_WAIT_S = 4.0;       // your “reduce delay to 4s”
    private boolean kicking = false;

    /* ===================== OUTTAKE ===================== */
    private int outtakeStep = 0;
    private boolean outtakeDone = false;

    /* ===================== HELPERS ===================== */
    private void setSpinner(double pos) {
        spinnerFar.setPosition(pos);
        spinnerClose.setPosition(pos);
    }

    private double getFlywheelRpm() {
        // flywheel.getVelocity() is ticks/sec
        return (flywheel.getVelocity() / TICKS_PER_REV) * 60.0;
    }

    private void buildPosesAndPaths() {
        double h = startPose.getHeading();

        backPose = new Pose(
                startPose.getX() - BACK_DISTANCE * Math.cos(h),
                startPose.getY() - BACK_DISTANCE * Math.sin(h),
                h
        );

        backPath = new Path(new BezierLine(startPose, backPose));
        backPath.setConstantHeadingInterpolation(h);
    }

    private void setState(int newState) {
        state = newState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }

    /* ===================== FLYWHEEL ===================== */
    private void startFlywheel() {
        kicking = true;
        flywheelTimer.resetTimer();
    }

    private void updateFlywheel() {
        if (!kicking) {
            flywheel.setVelocity(TARGET_TPS);
            return;
        }

        if (flywheelTimer.getElapsedTimeSeconds() < KICK_TIME_S) {
            flywheel.setPower(KICK_POWER);
        } else {
            kicking = false;
            flywheel.setVelocity(TARGET_TPS);
        }
    }

    /* ===================== OUTTAKE (RT-style, seconds-based) ===================== */
    private void updateOuttake() {

        intake.setPower(-1);
        double t = actionTimer.getElapsedTimeSeconds();

        if (outtakeStep == 0) { setSpinner(SPINNER_PRE); ejector.setPosition(EJECTOR_DOWN); if (t >= 0.40) outtakeStep++; return; }

        // Shot 1
        if (outtakeStep == 1) { ejector.setPosition(EJECTOR_UP);   if (t >= 0.80) outtakeStep++; return; }
        if (outtakeStep == 2) { ejector.setPosition(EJECTOR_DOWN); if (t >= 1.60) outtakeStep++; return; }
        if (outtakeStep == 3) { setSpinner(SPINNER_1);             if (t >= 2.00) outtakeStep++; return; }

        // Shot 2
        if (outtakeStep == 4) { ejector.setPosition(EJECTOR_UP);   if (t >= 2.40) outtakeStep++; return; }
        if (outtakeStep == 5) { ejector.setPosition(EJECTOR_DOWN); if (t >= 3.20) outtakeStep++; return; }
        if (outtakeStep == 6) { setSpinner(SPINNER_2);             if (t >= 3.60) outtakeStep++; return; }

        // Shot 3 + finish
        if (outtakeStep == 7) { ejector.setPosition(EJECTOR_UP);   if (t >= 4.00) outtakeStep++; return; }
        if (outtakeStep == 8) {
            ejector.setPosition(EJECTOR_DOWN);
            if (t >= 4.80) {
                setSpinner(SPINNER_0);
                intake.setPower(0);
                outtakeDone = true;
            }
        }
    }

    /* ===================== FSM ===================== */
    private void autonomousUpdate() {

        switch (state) {

            case START_FLYWHEEL:
                startFlywheel();
                setState(DRIVE_BACK);
                break;

            case DRIVE_BACK:
                follower.followPath(backPath, true);
                setState(WAIT_SPINUP);
                break;

            case WAIT_SPINUP: {
                if (follower.isBusy()) return;

                double rpm = getFlywheelRpm();
                boolean rpmReady = rpm >= (TARGET_RPM * RPM_READY_RATIO);
                boolean timeUp = actionTimer.getElapsedTimeSeconds() >= MAX_WAIT_S;

                // Condition-based wait (RPM), but with a hard cap of 4s
                if (rpmReady || timeUp) {
                    // Always wait an extra 1s after ready (your requirement)
                    setState(WAIT_EXTRA_1S);
                }
                break;
            }

            case WAIT_EXTRA_1S:
                if (actionTimer.getElapsedTimeSeconds() >= 1.0) {
                    actionTimer.resetTimer();
                    outtakeStep = 0;
                    outtakeDone = false;
                    setState(SHOOT_3);
                }
                break;

            case SHOOT_3:
                updateOuttake();
                if (outtakeDone) {

                    /* ===================== YOUR CUSTOM COMMANDS GO HERE =====================
                       Example ideas:
                       - setSpinner(SPINNER_PRE);
                       - intake.setPower(1);
                       - do a servo move
                       Keep it non-blocking: use timers and/or add a new state.
                    ========================================================================== */

                    // HARD STOP + end auto (no further movement)
                    follower.breakFollowing();   // if your Pedro version supports it; harmless if it exists
                    flywheel.setPower(0);
                    intake.setPower(0);
                    ejector.setPosition(EJECTOR_DOWN);
                    setSpinner(SPINNER_0);
                    setState(DONE);
                }
                break;

            default:
                // DONE
                flywheel.setPower(0);
                intake.setPower(0);
                // Optionally keep follower updating for pose telemetry only
                break;
        }
    }

    /* ===================== OPMODE ===================== */
    @Override
    public void init() {

        pathTimer = new Timer();
        actionTimer = new Timer();
        flywheelTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        ejector = hardwareMap.get(Servo.class, "ejector");
        spinnerFar = hardwareMap.get(Servo.class, "SpinnerFar");
        spinnerClose = hardwareMap.get(Servo.class, "SpinnerClose");

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        ejector.setPosition(EJECTOR_DOWN);
        setSpinner(SPINNER_0);
        intake.setPower(0);
        flywheel.setPower(0);

        buildPosesAndPaths();
        follower.setStartingPose(startPose);

        telemetry.addLine("Auto ready (STOP AFTER SHOOT).");
        telemetry.addData("startPose", startPose);
        telemetry.update();
    }

    @Override
    public void start() {
        setState(START_FLYWHEEL);
    }

    @Override
    public void loop() {
        follower.update();
        updateFlywheel();
        autonomousUpdate();

        telemetry.addData("state", state);
        telemetry.addData("pose", follower.getPose());
        telemetry.addData("flywheelRPM", getFlywheelRpm());
        telemetry.addData("wait/action s", actionTimer.getElapsedTimeSeconds());
        telemetry.addData("pathBusy", follower.isBusy());
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0);
        flywheel.setPower(0);
        ejector.setPosition(EJECTOR_DOWN);
        setSpinner(SPINNER_0);
    }
}