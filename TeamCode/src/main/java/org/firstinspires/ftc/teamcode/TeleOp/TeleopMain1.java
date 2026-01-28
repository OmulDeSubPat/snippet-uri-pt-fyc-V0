package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "testodo")
public class TeleopMain1 extends LinearOpMode {

    /* ================= CONSTANTS ================= */
    private static final double INCH_TO_CM = 2.54;

    /* ================= MOTORS ================= */
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;

    /* ================= LOCALIZATION ================= */
    private PinpointLocalizer pinpoint;
    private double pX_cm = 0;
    private double pY_cm = 0;
    private double headingDeg = 0;

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
        pinpoint.setStartPose(new Pose(0, 0, 0));
    }

    /* ================= DRIVE ================= */

    private void setWheelsPower() {
        double leftX  = gamepad2.left_stick_x;
        double leftY  = -gamepad2.left_stick_y; // forward is negative
        double rightX = gamepad2.right_stick_x;

        double fl = leftY + leftX + rightX;
        double bl = leftY - leftX + rightX;
        double fr = leftY - leftX - rightX;
        double br = leftY + leftX - rightX;

        double max = Math.max(
                Math.abs(fl),
                Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))
        );

        if (max > 1.0) {
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }

        front_left.setPower(fl);
        back_left.setPower(bl);
        front_right.setPower(fr);
        back_right.setPower(br);
    }

    /* ================= LOCALIZATION ================= */

    private void updateLocalization() {
        pinpoint.update();
        Pose pose = pinpoint.getPose();

        // Convert inches -> cm
        pX_cm = pose.getX() * INCH_TO_CM;
        pY_cm = pose.getY() * INCH_TO_CM;

        // Convert radians -> degrees
        headingDeg = Math.toDegrees(pose.getHeading());
    }

    /* ================= TELEMETRY ================= */

    private void updateTelemetry() {
        telemetry.addData("X (cm)", "%.2f", pX_cm);
        telemetry.addData("Y (cm)", "%.2f", pY_cm);
        telemetry.addData("Heading (deg)", "%.2f", headingDeg);
    }

    /* ================= OPMODE ================= */

    @Override
    public void runOpMode() {

        initWheels();
        initLocalization();

        telemetry.addLine("TeleOp Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            setWheelsPower();
            updateLocalization();
            updateTelemetry();
            telemetry.update();
        }
    }
}
