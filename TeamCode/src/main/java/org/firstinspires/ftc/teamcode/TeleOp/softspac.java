package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="robotPR")
public class softspac extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        DcMotor slide = hardwareMap.dcMotor.get("slide");


        Servo claw = hardwareMap.servo.get("claw");


        claw.setPosition(0.0);


        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        double ticks = 384.5;
        double target1 = 0;
        double turnage = 0;


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            boolean X = gamepad1.x;
            boolean circle = gamepad1.circle;
            boolean square = gamepad1.square;
            boolean triangle = gamepad1.triangle;
            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = gamepad1.left_stick_y;
            double rightStickX = gamepad1.right_stick_x;
            double rightStickY = gamepad1.right_stick_y;
            boolean RB = gamepad1.right_bumper;
            boolean LB = gamepad1.left_bumper;
            float LT = gamepad1.left_trigger;
            float RT = gamepad1.right_trigger;
            boolean s = gamepad1.dpad_up;
            boolean d = gamepad1.dpad_right;
            boolean st = gamepad1.dpad_left;
            boolean j = gamepad1.dpad_down;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = -(y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);




            if (LT > 0) {
                target1 = target1 + ((ticks / 360) * 5 * LT);
                if (target1 > 0) {
                    target1 = 0;
                }
                slide.setTargetPosition((int) target1);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
                telemetry.addData("LT", LT);
            }
            if (RT > 0) {
                target1 = target1 - ((ticks / 360) * 5 * RT);
                if (target1 < -3076) {
                    target1 = -3076;
                }
                slide.setTargetPosition((int) target1);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(0.5);
                telemetry.addData("RT", RT);
            }
            telemetry.addData("target1", target1);
            telemetry.addData("bl", backLeftPower);
            telemetry.addData("br", backRightPower);
            telemetry.addData("fl", frontLeftPower);
            telemetry.addData("fr", frontRightPower);
            telemetry.update();

            if (triangle) {
                claw.setPosition(0.05);
            }

            if (X) {
                claw.setPosition(0.4);

            }


        }
    }
}
