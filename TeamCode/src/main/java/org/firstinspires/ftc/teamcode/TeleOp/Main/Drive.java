package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Drive {

    //Motor sasiu
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;

    LinearOpMode opMode;

    public Drive(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    void SetWheelsPower() {
        double left_x  = opMode.gamepad1.left_stick_x;
        double left_y  = -opMode.gamepad1.left_stick_y; // forward is negative
        double right_x = opMode.gamepad1.right_stick_x;

        double front_left_pw  = left_y + left_x + right_x;
        double back_left_pw   = left_y - left_x + right_x;
        double front_right_pw = left_y - left_x - right_x;
        double back_right_pw  = left_y + left_x - right_x;

        double max = Math.max(Math.abs(front_left_pw),
                Math.max(Math.abs(back_left_pw),
                        Math.max(Math.abs(front_right_pw), Math.abs(back_right_pw))));

        if (max > 1.0) {
            front_left_pw  /= max;
            back_left_pw   /= max;
            front_right_pw /= max;
            back_right_pw  /= max;
        }

        front_left.setPower(front_left_pw);
        back_left.setPower(back_left_pw);
        front_right.setPower(front_right_pw);
        back_right.setPower(back_right_pw);
    }

    public void InitWheels() {
        front_left = opMode.hardwareMap.dcMotor.get("leftFront");
        front_right = opMode.hardwareMap.dcMotor.get("rightFront");
        back_left = opMode.hardwareMap.dcMotor.get("leftRear");
        back_right = opMode.hardwareMap.dcMotor.get("rightRear");

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void loop() {
        SetWheelsPower();
    }
}
