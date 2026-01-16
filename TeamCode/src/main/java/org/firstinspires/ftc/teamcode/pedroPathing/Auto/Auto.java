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

    private ElapsedTime timer;

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

    static final double TICKS_PER_REV = 384.5;

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

        timer = new ElapsedTime();
        timer.reset();
    }

    @Override
    public void start() {
        state = 0;
        timer.reset();
    }

    @Override
    public void loop() {

        flywheel.setPower(0.7);

        switch (state) {

            case 0: // Drive forward
                setDrivePower(0.5);
                timer.reset();
                state = 1;
                break;

            case 1: // Drive for 1.5 seconds
                if (timer.seconds() > 10) {
                    setDrivePower(0.0);
                    ejector.setPosition(0.285);
                    timer.reset();
                    state = 2;
                }
                break;

            case 2: // Eject wait
                if (timer.seconds() > 1.5) {
                    ejector.setPosition(0.005);
                    timer.reset();
                    state = 3;
                }
                break;

            case 3: // Spin spinner
                spinner.setTargetPosition((int)(120 * TICKS_PER_REV));
                spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spinner.setPower(0.5);
                timer.reset();
                state = 4;
                break;

            case 4: // Wait spinner
                if (!spinner.isBusy() || timer.seconds() > 1.5) {
                    ejector.setPosition(0.285);
                    timer.reset();
                    state = 5;
                }
                break;

            case 5: // Final eject
                if (timer.seconds() > 1.5) {
                    ejector.setPosition(0.005);
                    state = 6;
                }
                break;

            case 6:
                // DONE
                break;
        }
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
