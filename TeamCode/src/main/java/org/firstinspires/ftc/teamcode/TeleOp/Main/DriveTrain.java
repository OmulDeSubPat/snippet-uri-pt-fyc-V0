package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    //Motor sasiu
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;

    private final Gamepad gamepad;

    public DriveTrain(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;

        front_left = hardwareMap.dcMotor.get("leftFront");
        front_right = hardwareMap.dcMotor.get("rightFront");
        back_left = hardwareMap.dcMotor.get("leftRear");
        back_right = hardwareMap.dcMotor.get("rightRear");

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void runDrive() {
        while (true) {  // va rula cât timp thread-ul e activ (controlat din TeleOpMain)
            double left_x  = gamepad.left_stick_x;
            double left_y  = -gamepad.left_stick_y; // forward is negative
            double right_x = gamepad.right_stick_x;

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
    }
}