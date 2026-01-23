package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "AutoAIMBlue1")
public class codisi extends LinearOpMode {

    // Motors
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;

    private DcMotorEx tureta;
    double LEFT_LIMIT = -100;   // degrees
    double RIGHT_LIMIT = 100;  // degrees

    // Localization
    private PinpointLocalizer pinpoint;
    private double pX = 0;
    private double pY = 0;
    private double distanceStart = 0;
    private Pose startPose;
    private double headingFromStart = 0;
    private double headingDifference;

    //variables

    private double DEG_PER_TICK = 360.0 / 560.0;

    private double xC;
    private double yC;
    private double rC;
    private double cateta;
    private double unghiRobot;
    private double deg2rad = 180/3.141592654;


    /* ================= INIT ================= */

    private void initWheels() {
        front_left  = hardwareMap.dcMotor.get("lf");
        front_right = hardwareMap.dcMotor.get("rf");
        back_left   = hardwareMap.dcMotor.get("lr");
        back_right  = hardwareMap.dcMotor.get("rr");

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void initLocalization() {
        pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        pinpoint.setStartPose(startPose = new Pose(9, 9, 90));
    }

    /* ================= DRIVE ================= */


    /* ================= LOCALIZATION ================= */
    private double limitAngle(double targetDeg) {
        double currentDeg = tureta.getCurrentPosition() * DEG_PER_TICK;

        if (currentDeg >= RIGHT_LIMIT && targetDeg > currentDeg) {
            return RIGHT_LIMIT;
        }
        if (currentDeg <= LEFT_LIMIT && targetDeg < currentDeg) {
            return LEFT_LIMIT;
        }
        return targetDeg;
    }

    private void updateLocalization() {
        pinpoint.update();
        Pose pose = pinpoint.getPose();
        pX = pose.getX();
        pY = pose.getY();
        distanceStart = pose.distanceFrom(startPose);
        headingFromStart = pinpoint.getPose().getHeading();

    }
    private void distanceRobot() {
        pinpoint.update();

        rC = Math.sqrt(
                Math.pow(xC - pX, 2) +
                        Math.pow(yC - pY, 2)
        );

        if (rC == 0) return; // safety

        // pick correct cathetus
        if (Math.abs(xC - pX) > Math.abs(yC - pY)) {
            cateta = yC - pY;
        } else {
            cateta = xC - pX;
        }

        // angle in degrees
        unghiRobot = Math.toDegrees(Math.asin(cateta / rC));
    }


    /* ================= TELEMETRY ================= */

    private void updateTelemetry() {
        telemetry.addData("X", "%.2f", pX);
        telemetry.addData("Y", "%.2f", pY);
        telemetry.addData("Heading (rad)", "%.2f", Math.toDegrees(pinpoint.getPose().getHeading()));
        telemetry.addData("headingtureta", tureta.getCurrentPosition() * DEG_PER_TICK);
    }

    /* ================= OPMODE ================= */
    public void Rotate(double targetDeg) {
        final double TICKS_PER_REV = 384.5;
        final double MAX_TPS = (431.0 / 60.0) * TICKS_PER_REV;

        targetDeg = limitAngle(targetDeg);

        int targetTicks = (int) Math.round(targetDeg / DEG_PER_TICK);

        tureta.setTargetPosition(targetTicks);
        tureta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tureta.setVelocity(MAX_TPS);
    }


    @Override
    public void runOpMode() {
        xC = 130;   // example
        yC = 0;

        initWheels();
        initLocalization();
        tureta = hardwareMap.get(DcMotorEx.class, "tureta");
        tureta.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        tureta.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        tureta.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("TeleOp Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updateLocalization();
            updateTelemetry();
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
            distanceRobot();

            double headingDeg = Math.toDegrees(pinpoint.getPose().getHeading());
            Rotate(unghiRobot - headingDeg);


            telemetry.update();
        }
    }
}