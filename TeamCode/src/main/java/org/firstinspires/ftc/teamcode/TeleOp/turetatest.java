package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TestareTureta")
public class turetatest extends LinearOpMode {

    /* ===================== DRIVE ===================== */
    private DcMotor front_left, front_right, back_left, back_right;

    private void InitWheels() {
        front_left  = hardwareMap.dcMotor.get("lf");
        front_right = hardwareMap.dcMotor.get("rf");
        back_left   = hardwareMap.dcMotor.get("lr");
        back_right  = hardwareMap.dcMotor.get("rr");

        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void SetWheelsPower() {
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double fl = ly + lx + rx;
        double bl = ly - lx + rx;
        double fr = ly - lx - rx;
        double br = ly + lx - rx;

        double max = Math.max(Math.abs(fl),
                Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br))));

        if (max > 1.0) { fl /= max; bl /= max; fr /= max; br /= max; }

        front_left.setPower(fl);
        back_left.setPower(bl);
        front_right.setPower(fr);
        back_right.setPower(br);
    }

    /* ===================== TURRET ===================== */
    private DcMotorEx tureta;

    private static final double MOTOR_TICKS_PER_REV = 384.5;
    private static final double MOTOR_TO_TURRET_RATIO = 76.0 / 24.0;
    private static final double DEG_PER_TICK_TURETA =
            360.0 / (MOTOR_TICKS_PER_REV * MOTOR_TO_TURRET_RATIO);

    private static final double LEFT_LIMIT  = -110;
    private static final double RIGHT_LIMIT = 110;

    private static final double kP = 0.015;
    private static final double MAX_POWER_TURETA = 0.25;
    private static final double DEADBAND_DEG = 1.5;

    // IMPORTANT CONVENTION:
    // 0 deg turret = LAUNCH direction (BACK of robot).
    // If encoder=0 happens when turret physically points BACK, then offset is 0.
    private static final double TURRET_ZERO_OFFSET_DEG = 0.0;

    private boolean aimingEnabled = false;

    private double currentTurretDeg = 0;
    private double targetTurretDeg = 0;
    private double errorDeg = 0;
    private double power = 0;

    /* ===================== LOCALIZATION ===================== */
    private PinpointLocalizer pinpoint;
    private Pose pose;

    /* ===================== TARGET (field inches) ===================== */
    private double targetX = 12;
    private double targetY = 136;

    // If you don't know turret offsets, keep 0 for testing aim
    private final double turretOffsetX = 0.0;
    private final double turretOffsetY = 0.0;
    private double turretX, turretY;

    /* ===================== TELEMETRY ===================== */
    private final int telemetryDelayMs = 120;
    private ElapsedTime telemetryTimer = new ElapsedTime();

    private double normalizeAngle(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    // Returns turret angle relative to robot, where 0 deg = BACK (launch direction)
    private double getCurrentTurretDegRobotFrame() {
        return normalizeAngle(tureta.getCurrentPosition() * DEG_PER_TICK_TURETA + TURRET_ZERO_OFFSET_DEG);
    }

    private void initAll() {
        InitWheels();

        tureta = hardwareMap.get(DcMotorEx.class, "tureta");
        tureta.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        tureta.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        pinpoint.setStartPose(new Pose(22, 127, Math.toRadians(-36)));
    }

    private void updateTurretAimOrHold() {
        if (pose == null) return;

        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeadingRad = pose.getHeading();
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        // Turret position on field (if offset from robot center)
        turretX = robotX + turretOffsetX * Math.cos(robotHeadingRad) - turretOffsetY * Math.sin(robotHeadingRad);
        turretY = robotY + turretOffsetX * Math.sin(robotHeadingRad) + turretOffsetY * Math.cos(robotHeadingRad);

        currentTurretDeg = getCurrentTurretDegRobotFrame(); // 0=BACK

        if (aimingEnabled) {
            double dx = targetX - turretX;
            double dy = targetY - turretY;

            // Angle to target in FIELD frame
            double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

            // Convert to ROBOT-FORWARD-relative:
            // forwardRel = fieldAngle - robotHeading
            // But turret frame is 0 at BACK, so subtract 180 to express target in turret frame.
            targetTurretDeg = normalizeAngle(fieldAngleDeg - robotHeadingDeg - 180.0);

            // Clip to allowed turret mechanical range (in turret frame, 0=BACK)
            targetTurretDeg = Range.clip(targetTurretDeg, LEFT_LIMIT, RIGHT_LIMIT);
        }
        // else: HOLD last targetTurretDeg

        errorDeg = normalizeAngle(targetTurretDeg - currentTurretDeg);

        if (Math.abs(errorDeg) < DEADBAND_DEG) {
            tureta.setPower(0);
            power = 0;
            return;
        }

        power = Range.clip(errorDeg * kP, -MAX_POWER_TURETA, MAX_POWER_TURETA);
        tureta.setPower(power);
    }

    private void updateTelemetry() {
        if (telemetryTimer.milliseconds() < telemetryDelayMs) return;

        telemetry.addData("AIM (Y/triangle)", aimingEnabled);
        telemetry.addData("TargetXY", "(%.1f, %.1f)", targetX, targetY);

        if (pose != null) {
            telemetry.addData("PoseX", "%.1f", pose.getX());
            telemetry.addData("PoseY", "%.1f", pose.getY());
            telemetry.addData("HeadingDeg", "%.1f", Math.toDegrees(pose.getHeading()));
        }

        telemetry.addData("TurretDeg(curr) [0=BACK]", "%.1f", currentTurretDeg);
        telemetry.addData("TurretDeg(tgt)  [0=BACK]", "%.1f", targetTurretDeg);
        telemetry.addData("ErrDeg", "%.1f", errorDeg);
        telemetry.addData("Power", "%.2f", power);

        telemetry.update();
        telemetryTimer.reset();
    }

    @Override
    public void runOpMode() {
        initAll();

        telemetry.addLine("READY:");
        telemetry.addLine("Drive: left stick + right stick X");
        telemetry.addLine("Y (Triangle) = toggle AIM");
        telemetry.addLine("Dpad = move target point (test)");
        telemetry.addLine("Back/Share = reset turret encoder (careful)");
        telemetry.addLine("Turret angle convention: 0 deg = BACK (launch direction)");
        telemetry.update();

        waitForStart();

        // Warmup pinpoint + capture hold => turret won't jump at Play
        for (int i = 0; i < 6 && opModeIsActive(); i++) {
            pinpoint.update();
            pose = pinpoint.getPose();
            sleep(20);
        }

        aimingEnabled = false;
        targetTurretDeg = getCurrentTurretDegRobotFrame(); // hold where it is

        while (opModeIsActive()) {

            // DRIVE
            SetWheelsPower();

            // POSE
            pinpoint.update();
            pose = pinpoint.getPose();

            // Toggle aiming (PS triangle often maps to Y)
            if (gamepad1.yWasPressed()) {
                aimingEnabled = !aimingEnabled;
                if (!aimingEnabled) targetTurretDeg = getCurrentTurretDegRobotFrame();
            }

            // Move target around (NO debounce - as requested)
            if (gamepad1.dpad_up)    targetY += 0.5;
            if (gamepad1.dpad_down)  targetY -= 0.5;
            if (gamepad1.dpad_right) targetX += 0.5;
            if (gamepad1.dpad_left)  targetX -= 0.5;

            // Reset encoder (as-is - NOT the "safe reset" version)
            if (gamepad1.backWasPressed() || gamepad1.shareWasPressed()) {
                tureta.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                tureta.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                targetTurretDeg = getCurrentTurretDegRobotFrame();
            }

            // TURRET
            updateTurretAimOrHold();

            // TELEMETRY
            updateTelemetry();
        }
    }
}
