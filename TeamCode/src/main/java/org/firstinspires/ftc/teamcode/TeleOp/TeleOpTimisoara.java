package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOpServosAndPivoter")
public class TeleOpTimisoara extends LinearOpMode {

    Servo clawL;
    Servo test;
    DcMotor worm; // motor 233 ratio 28:1
    final double ticksWorm = 751.8;
    double targetWorm = 0;
    double unghiWorm = 0;
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;
    Servo clawR;
    Servo flip;
    DcMotor pivoter; // 60 rpm
    DcMotor glisiera;
    int targetGlisiera = 0;

    double ticksPivoter = 2786.2;
    double unghiPivoter = 0;
    double targetPivoter = 0;

    // Mode switch
    boolean modController = true;

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

    private void SetWheelsPower() {
        double left_x  = gamepad1.left_stick_x;
        double left_y  = -gamepad1.left_stick_y; // forward is negative
        double right_x = gamepad1.right_stick_x;

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

    private void servoInit() {
        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");
        clawR.setDirection(Servo.Direction.REVERSE);
        flip = hardwareMap.servo.get("flip");
    }

    private void pozInit() {
        flip.setPosition(0.5);//servo
        unghiPivoter = -139.4;
        targetPivoter = (unghiPivoter / 360.0) * ticksPivoter;
        pivoter.setTargetPosition((int) targetPivoter);
        pivoter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivoter.setPower(0.4);

        glisiera.setTargetPosition(-10);//0
        glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        glisiera.setPower(0.4);
    }


    private void DcInit() {
        pivoter = hardwareMap.dcMotor.get("pivoter");
        pivoter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivoter.setTargetPosition(0);
        pivoter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivoter.setPower(0);

        glisiera = hardwareMap.dcMotor.get("glisiera");
        glisiera.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        worm = hardwareMap.dcMotor.get("worm");
        worm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void runOpMode() {
        servoInit();
        DcInit();
        pozInit();
        InitWheels();

        waitForStart();

        // --- Thread for wheels / drivetrain ---
        Thread wheelThread = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                if (modController) {
                    handleWheelControlsMode1();
                } else {
                    SetWheelsPower();
                    clawMovement();
                }
            }
        });

        // --- Thread for other systems ---
        Thread systemsThread = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                UpdateTelemetry();

                if (modController) {
                    glisieraMovementMode1();
                    pivoterMovementMode1();
                    clawMovementMode1();
                } else {
                    glisieraMovement();
                    pozitiiScore();
                    intake();
                    pivoterMovement();
                }

                // Toggle mode with PS button
                if (gamepad1.ps) {
                    modController = !modController;
                    sleep(300); // debounce
                }

            }
        });

        wheelThread.start();
        systemsThread.start();

        // 🔑 Keep the main thread alive
        while (opModeIsActive() && !isStopRequested()) {
            idle(); // yields to avoid watchdog warnings
        }
    }


    // --- Mode 1 Wheel Controls ---
    private void handleWheelControlsMode1() {
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;

        double rot = 0;
        if (gamepad1.dpad_left) rot = -0.3;
        if (gamepad1.dpad_right) rot = 0.3;

        double front_left_pw  = ly + lx + rot;
        double back_left_pw   = ly - lx + rot;
        double front_right_pw = ly - lx - rot;
        double back_right_pw  = ly + lx - rot;

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

    // --- Mode 1 Glisiera ---
    private static final int TOP_POSITION = -3000;
    private static final int BOTTOM_POSITION = -4;
    private static final double MOTOR_POWER = 0.6;

    private void glisieraMovementMode1() {
        double stickY = gamepad1.right_stick_y;
        if (Math.abs(stickY) > 0.1) {
            targetGlisiera += (int) (stickY * 20);
            if (targetGlisiera < TOP_POSITION) targetGlisiera = TOP_POSITION;
            if (targetGlisiera > BOTTOM_POSITION) targetGlisiera = BOTTOM_POSITION;

            glisiera.setTargetPosition(targetGlisiera);
            glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            glisiera.setPower(MOTOR_POWER);
        }
    }

    // --- Mode 1 Pivoter ---
    private void pivoterMovementMode1() {
        if (gamepad1.right_bumper) {
            unghiPivoter += 0.2;
        }
        if (gamepad1.right_trigger > 0.5) {
            unghiPivoter -= 0.2;
        }
        targetPivoter = (unghiPivoter / 360.0) * ticksPivoter;
        pivoter.setTargetPosition((int) targetPivoter);
        pivoter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivoter.setPower(0.4);
    }

    // --- Mode 1 Claw ---
    private void clawMovementMode1() {
        if (gamepad1.circle) {
            clawL.setPosition(0.0);
            clawR.setPosition(0.0);
        }
        if (gamepad1.cross) {
            clawL.setPosition(0.38);
            clawR.setPosition(0.38);
        }
        if(gamepad1.square) {//score
            unghiPivoter = -49.6;
            targetPivoter = (unghiPivoter / 360.0) * ticksPivoter;
            pivoter.setTargetPosition((int) targetPivoter);
            pivoter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivoter.setPower(0.4);

            flip.setPosition(0.07);
            targetGlisiera = -750;
            glisiera.setTargetPosition(targetGlisiera);
            glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            glisiera.setPower(0.4);
        }
        if(gamepad1.triangle) {//intake
            flip.setPosition(0.5);
            unghiPivoter = -139.4;
            targetPivoter = (unghiPivoter / 360.0) * ticksPivoter;
            pivoter.setTargetPosition((int) targetPivoter);
            pivoter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivoter.setPower(0.4);

            glisiera.setTargetPosition(-10);
            glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            glisiera.setPower(0.4);
        }
    }

    // --- Existing Controls (Mode 2) ---
    private void glisieraMovement() {
        double stickY = gamepad2.left_stick_y;
        if (Math.abs(stickY) > 0.1) {
            targetGlisiera += (int) (stickY * 20);
            if (targetGlisiera < TOP_POSITION) targetGlisiera = TOP_POSITION;
            if (targetGlisiera > BOTTOM_POSITION) targetGlisiera = BOTTOM_POSITION;

            glisiera.setTargetPosition(targetGlisiera);
            glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            glisiera.setPower(MOTOR_POWER);
        }
    }

    private void pivoterMovement() {
        if(gamepad2.right_bumper) {
            unghiPivoter -= 0.2;
        }
        if(gamepad2.left_bumper) {
            unghiPivoter += 0.2;
        }
        targetPivoter = (unghiPivoter / 360.0) * ticksPivoter;
        pivoter.setTargetPosition((int) targetPivoter);
        pivoter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivoter.setPower(0.4);


    }

    private void UpdateTelemetry() {
        telemetry.addData("Mode", modController ? "Mode 1 (Driver)" : "Mode 2 (Default)");
        telemetry.addData("pivoter position:", pivoter.getCurrentPosition());
        telemetry.addData("clawL position:", clawL.getPosition());
        telemetry.addData("clawR position:", clawR.getPosition());
        telemetry.addData("flip position:", flip.getPosition());
        telemetry.addData("unghiPivoter:", unghiPivoter);
        telemetry.update();
    }

    private void clawMovement() {
        if (gamepad2.dpad_up) {
            clawL.setPosition(0.38);
            clawR.setPosition(0.38);
        }
        if (gamepad2.dpad_down) {
            clawL.setPosition(0.0);
            clawR.setPosition(0.0);
        }
    }

    private void intake() {
        if(gamepad2.triangle)
        {
            flip.setPosition(0.5);
            unghiPivoter = -139.4;
            targetPivoter = (unghiPivoter / 360.0) * ticksPivoter;
            pivoter.setTargetPosition((int) targetPivoter);
            pivoter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivoter.setPower(0.4);

            glisiera.setTargetPosition(-10);
            glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            glisiera.setPower(0.4);
        }
    }

    private void pozitiiScore() {
        if(gamepad2.circle) {
            flip.setPosition(0.29);
            unghiPivoter = -100;
            targetPivoter = (unghiPivoter / 360.0) * ticksPivoter;
            pivoter.setTargetPosition((int) targetPivoter);
            pivoter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivoter.setPower(0.4);

            glisiera.setTargetPosition(-10);
            glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            glisiera.setPower(0.4);
        }
        if(gamepad2.cross) {
            unghiPivoter = -75.6;
            targetPivoter = (unghiPivoter / 360.0) * ticksPivoter;
            pivoter.setTargetPosition((int) targetPivoter);
            pivoter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivoter.setPower(0.4);

            flip.setPosition(0.17);
            glisiera.setTargetPosition(-10);
            glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            glisiera.setPower(0.4);
        }
        if(gamepad2.square) {
            unghiPivoter = -49.6;
            targetPivoter = (unghiPivoter / 360.0) * ticksPivoter;
            pivoter.setTargetPosition((int) targetPivoter);
            pivoter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivoter.setPower(0.4);

            flip.setPosition(0.07);
            targetGlisiera = -150;
            glisiera.setTargetPosition(targetGlisiera);
            glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            glisiera.setPower(0.4);
        }
        if(gamepad2.share) {
            unghiPivoter = -49.6;
            targetPivoter = (unghiPivoter / 360.0) * ticksPivoter;
            pivoter.setTargetPosition((int) targetPivoter);
            pivoter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivoter.setPower(0.4);

            flip.setPosition(0.07);
            targetGlisiera = -750;
            glisiera.setTargetPosition(targetGlisiera);
            glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            glisiera.setPower(0.4);
        }
        if(gamepad1.touchpad) {
            unghiPivoter = 0;
            targetPivoter = (unghiPivoter / 360.0) * ticksPivoter;
            pivoter.setTargetPosition((int) targetPivoter);
            pivoter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivoter.setPower(0.4);

            flip.setPosition(0.33);
        }
    }
}
