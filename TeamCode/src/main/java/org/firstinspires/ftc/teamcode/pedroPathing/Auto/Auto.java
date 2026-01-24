package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "All Motors Fixed", group = "Test")
public class Auto extends OpMode {

    private ElapsedTime timer = new ElapsedTime();
    private int state = 0;

    DcMotorEx spinner;
    DcMotor intake;
    DcMotorEx tureta;
    DcMotor flywheel;

    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;

    Servo ejector;


    @Override
    public void init() {

        front_left  = hardwareMap.dcMotor.get("lf");
        front_right = hardwareMap.dcMotor.get("rf");
        back_left   = hardwareMap.dcMotor.get("lr");
        back_right  = hardwareMap.dcMotor.get("rr");

        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);

        spinner  = hardwareMap.get(DcMotorEx.class, "spinner");
        intake   = hardwareMap.get(DcMotor.class, "intake");
        tureta   = hardwareMap.get(DcMotorEx.class, "tureta");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        ejector  = hardwareMap.get(Servo.class, "ejector");

        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {
        state = 0;
        timer.reset();
    }

    @Override
    public void loop() {

        switch (state) {

            case 0:
                if (timer.seconds() >= 29.0) {
                    timer.reset();
                    state = 1;
                }
                break;

            case 1:
                setDrivePower(0.4);
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Time", timer.seconds());
        telemetry.update();
    }

    private void setDrivePower(double power) {
        front_left.setPower(power);
        front_right.setPower(power);
        back_left.setPower(power);
        back_right.setPower(power);
    }

    @Override
    public void stop() {
        setDrivePower(0);
        spinner.setPower(0);
        intake.setPower(0);
        tureta.setPower(0);
        flywheel.setPower(0);
    }
}
