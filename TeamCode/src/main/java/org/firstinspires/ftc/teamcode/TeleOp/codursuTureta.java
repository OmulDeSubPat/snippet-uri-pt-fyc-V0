package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "AutoAIMBlue")
public class codursuTureta extends LinearOpMode {

    /* ================= HARDWARE ================= */

    private DcMotor front_left, front_right, back_left, back_right;
    private DcMotorEx tureta;

    /* ================= CONSTANTS ================= */

    // Encoder + gearing
    private static final double MOTOR_TICKS_PER_REV = 384.5;
    private static final double MOTOR_TO_TURRET_RATIO = 76.0 / 24.0;

    private static final double DEG_PER_TICK =
            360.0 / (MOTOR_TICKS_PER_REV * MOTOR_TO_TURRET_RATIO);

    // Turret soft limits (degrees)
    private static final double LEFT_LIMIT  = -110;
    private static final double RIGHT_LIMIT = 110;

    // Control
    private static final double kP = 0.015;
    private static final double MAX_POWER = 0.2;

    /* ================= LOCALIZATION ================= */

    private PinpointLocalizer pinpoint;
    private Pose startPose;

    private double pX, pY;

    /* ================= TARGET ================= */

    // Example target (field coordinates)
    private double xC = 0;
    private double yC = 144;

    /* ================= INIT ================= */

    private void initWheels() {
        front_left  = hardwareMap.dcMotor.get("lf");
        front_right = hardwareMap.dcMotor.get("rf");
        back_left   = hardwareMap.dcMotor.get("lr");
        back_right  = hardwareMap.dcMotor.get("rr");

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void initLocalization() {
        pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        startPose = new Pose(22, 127, Math.toRadians(-36));
        pinpoint.setStartPose(startPose);
    }

    private void initTurret() {
        tureta = hardwareMap.get(DcMotorEx.class, "tureta");
        tureta.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        tureta.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        tureta.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /* ================= MATH ================= */

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /* ================= TURRET AIM ================= */

    private void updateTurretAim() {

        Pose pose = pinpoint.getPose();
        pX = pose.getX();
        pY = pose.getY();

        // Vector robot -> target (field frame)
        double dx = xC - pX;
        double dy = yC - pY;

        // Absolute field angle to target
        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));

        // Robot heading
        double robotHeading = Math.toDegrees(pose.getHeading());
        double currentTurretDeg = tureta.getCurrentPosition() * DEG_PER_TICK-180.0;
        double targetTurretDeg = normalizeAngle(fieldAngle - robotHeading);
        // Desired turret angle (robot frame, UNCLIPPED)
        if(Math.abs(targetTurretDeg) < RIGHT_LIMIT){
            targetTurretDeg = -180;
        }

        // Current turret angle


        currentTurretDeg=normalizeAngle(currentTurretDeg);
        // Error
        double error = normalizeAngle(targetTurretDeg - currentTurretDeg);

        // P control
        double power = error * kP;

        // ====== LIMIT SAFETY (THIS FIXES THE JUMPING) ======

//        if (currentTurretDeg <= RIGHT_LIMIT && power > 0) {
//            power = 0;
//        }
//        if (currentTurretDeg >= LEFT_LIMIT && power < 0) {
//            power = 0;
//        }


        power = Range.clip(power, -MAX_POWER, MAX_POWER);
        tureta.setPower(power);

        telemetry.addData("Target (raw)", "%.1f", targetTurretDeg);
        telemetry.addData("Turret", "%.1f", currentTurretDeg);
        telemetry.addData("Error", "%.1f", error);
        telemetry.addData("Power", "%.2f", power);
    }

    /* ================= DRIVE ================= */

    private void drive() {
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double fl = ly + lx + rx;
        double bl = ly - lx + rx;
        double fr = ly - lx - rx;
        double br = ly + lx - rx;

        double max = Math.max(Math.abs(fl),
                Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br))));

        if (max > 1.0) {
            fl /= max; bl /= max; fr /= max; br /= max;
        }

        front_left.setPower(fl);
        back_left.setPower(bl);
        front_right.setPower(fr);
        back_right.setPower(br);
    }


    /* ================= OPMODE ================= */

    @Override
    public void runOpMode() {

        initWheels();
        initLocalization();
        initTurret();

        telemetry.addLine("AutoAim Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            pinpoint.update();
            drive();
            updateTurretAim();

            telemetry.addData("X", "%.1f", pX);
            telemetry.addData("Y", "%.1f", pY);
            telemetry.addData("Heading", "%.1f",
                    Math.toDegrees(pinpoint.getPose().getHeading()));

            telemetry.update();
        }
    }
}