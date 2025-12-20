package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="sasiu")
public class sasiu extends LinearOpMode {
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;


    private void InitWheels()
    {
        front_left = hardwareMap.dcMotor.get("lf");
        front_right = hardwareMap.dcMotor.get("rf");
        back_left = hardwareMap.dcMotor.get("lr");
        back_right = hardwareMap.dcMotor.get("rr");

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    private void SetWheelsPower() {
        double left_x  = gamepad1.left_stick_x;
        double left_y  = -gamepad1.left_stick_y; // forward is negative
        double right_x = gamepad1.right_stick_x;

        // Calculate raw motor powers
        double front_left_pw  = left_y + left_x + right_x;
        double back_left_pw   = left_y - left_x + right_x;
        double front_right_pw = left_y - left_x - right_x;
        double back_right_pw  = left_y + left_x - right_x;

        // Normalize so no motor power exceeds 1.0
        double max = Math.max(Math.abs(front_left_pw),
                Math.max(Math.abs(back_left_pw),
                        Math.max(Math.abs(front_right_pw), Math.abs(back_right_pw))));

        if (max > 1.0) {
            front_left_pw  /= max;
            back_left_pw   /= max;
            front_right_pw /= max;
            back_right_pw  /= max;
        }

        // Apply power
        front_left.setPower(front_left_pw);
        back_left.setPower(back_left_pw);
        front_right.setPower(front_right_pw);
        back_right.setPower(back_right_pw);
    }




    public void runOpMode()
    {
        waitForStart();

        InitWheels();


        if (isStopRequested()) return;

        while(opModeIsActive() && !isStopRequested()) {
            SetWheelsPower();
            UpdateTelemetry();
        }
    }

    private void UpdateTelemetry()
    {
        telemetry.addData("roata Rstanga :", back_left.getPower());
        telemetry.addData("roata Rdreapta :", back_right.getPower());
        telemetry.addData("roata Fstanga :", front_left.getPower());
        telemetry.addData("roata Fdreapta :", front_right.getPower());


        telemetry.update();
    }

}