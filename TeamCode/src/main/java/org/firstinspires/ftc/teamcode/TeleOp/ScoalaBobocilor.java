package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="sasiu")
public abstract class ScoalaBobocilor extends LinearOpMode {
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;
    DcMotor pivoter;
    DcMotor glisiera_intake;
    DcMotor glisiera_outtake;
    CRServo intake_left;
    CRServo intake_right;
    Servo cleste;
    Servo axa_cleste;
    Servo unghi_intake_left;
    Servo unghi_intake_right;

    private void InitWheels() {
        front_left = hardwareMap.dcMotor.get("leftFront");
        front_right = hardwareMap.dcMotor.get("rightFront");
        back_left = hardwareMap.dcMotor.get("leftRear");
        back_right = hardwareMap.dcMotor.get("rightRear");

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void InitDc() {
        pivoter = hardwareMap.dcMotor.get("pivoter");
        glisiera_intake = hardwareMap.dcMotor.get("glisieraintake");
        glisiera_intake = hardwareMap.dcMotor.get("glisieraouttake");
        glisiera_intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        glisiera_intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    private void InitServos() {
        intake_right = hardwareMap.crservo.get("intakeright");
        //...///
    }

    private void SetWheelsPower() {
        double left_x = gamepad1.left_stick_x;
        double left_y = -gamepad1.left_stick_y; // forward is negative
        double right_x = gamepad1.right_stick_x;

        double front_left_pw = left_y + left_x + right_x;
        double back_left_pw = left_y - left_x + right_x;
        double front_right_pw = left_y - left_x - right_x;
        double back_right_pw = left_y + left_x - right_x;

        double max = Math.max(Math.abs(front_left_pw),
                Math.max(Math.abs(back_left_pw),
                        Math.max(Math.abs(front_right_pw), Math.abs(back_right_pw))));

        if (max > 1.0) {
            front_left_pw /= max;
            back_left_pw /= max;
            front_right_pw /= max;
            back_right_pw /= max;
        }

        front_left.setPower(front_left_pw);
        back_left.setPower(back_left_pw);
        front_right.setPower(front_right_pw);
        back_right.setPower(back_right_pw);
    }

    public void pozitieIntake()
    {
        glisiera_intake.setTargetPosition(384/2);
        glisiera_intake.setPower(0.8);
        glisiera_intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        glisiera_intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void pozitieOuttake(){
        glisiera_outtake.setTargetPosition((int) (384*3.75));
        glisiera_outtake.setPower(0.8);
        glisiera_outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        glisiera_outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
}
