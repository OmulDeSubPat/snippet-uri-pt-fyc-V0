package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "FlywheelTest")
public class flywheeltest extends LinearOpMode {

    // ================== FLYWHEEL CONSTANTS ==================
    static final double FLYWHEEL_TICKS_PER_REV = 28.0;    // REV encoder ticks per rev
    static double TARGET_RPM = 3600;              // Desired flywheel speed
    static double TARGET_TPS = TARGET_RPM * FLYWHEEL_TICKS_PER_REV / 60.0; // ticks/sec

    // PIDF coefficients tuned for stable velocity
    static double kP_v = 10.0;
    static final double kI_v = 0.0;
    static final double kD_v = 0.0;
    static double kF_v = 13.0;
    Servo trajectoryAngleModifier;

    DcMotorEx flywheel;
    DcMotor intake;
    Servo ejector;
    final double ejectorDown = 0.214;
    final double ejectorUp = 0.03;
    boolean intakeMode = false;
    boolean outtakeMode = false;

    private ElapsedTime spinnerTimeout = new ElapsedTime();
    private ElapsedTime outtakeTimeout = new ElapsedTime();
    double prev_t = 0;
    boolean step1Done = false;
    boolean turetaDisabled = false;
    boolean step2Done = false;
    boolean step3Done = false;
    boolean step4Done = false;
    boolean step5Done = false;
    boolean step6Done = false;
    boolean step7Done = false;
    boolean step8Done = false;
    boolean step9Done = false;
    boolean step10Done = false;
    boolean step11Done = false;
    boolean spinIntake = false;
    Double Posspinner=0.0;
    Servo spinnerFar;
    Servo spinnerCLose;
    private static final double SPINDEXER_OUTTAKE_OFFSET = -0.015; // your updated value
    private static final double SPINDEXER_INTAKE_OFFSET  = -0.01; // your old intake bias
    private static final double SPINNER_LAUNCH_POS       = 0.085; // launch/prep position

    private void runOuttake() {
        intake.setPower(-1);

        final int EJECTOR_UP_DELAY = 300;
        final int EJECTOR_DOWN_DELAY = 170;
        final int SPINNER_SLOT_CHANGE_DELAY = 300;
        final int INITIAL_DELAY = 400;  // <-- lowered from 600 to 400


        double t = outtakeTimeout.milliseconds();

        if (t - prev_t >= 10 && !step1Done) {
            Posspinner = 0.085;
            step1Done = true;
            prev_t = 10;
        }

        if (t >= prev_t + INITIAL_DELAY && !step2Done && step1Done) {
            ejector.setPosition(ejectorUp);
            step2Done = true;
            prev_t += INITIAL_DELAY;
        }

        if (t >= prev_t + EJECTOR_UP_DELAY && !step3Done && step2Done) {
            ejector.setPosition(ejectorDown);
            step3Done = true;
            prev_t += EJECTOR_UP_DELAY;
        }

        if (t >= prev_t + EJECTOR_DOWN_DELAY && !step4Done && step3Done) {
            Posspinner = 0.28;
            step4Done = true;
            prev_t += EJECTOR_DOWN_DELAY;
        }

        if (t >= prev_t + SPINNER_SLOT_CHANGE_DELAY && !step5Done && step4Done) {
            ejector.setPosition(ejectorUp);
            step5Done = true;
            prev_t += SPINNER_SLOT_CHANGE_DELAY;
        }

        if (t >= prev_t + EJECTOR_UP_DELAY && !step6Done && step5Done) {
            ejector.setPosition(ejectorDown);
            step6Done = true;
            prev_t += EJECTOR_UP_DELAY;
        }

        if (t >= prev_t + EJECTOR_DOWN_DELAY && !step7Done && step6Done) {
            Posspinner = 0.46;
            step7Done = true;
            prev_t += EJECTOR_DOWN_DELAY;
        }

        if (t >= prev_t + SPINNER_SLOT_CHANGE_DELAY && !step8Done && step7Done) {
            ejector.setPosition(ejectorUp);
            step8Done = true;
            prev_t += SPINNER_SLOT_CHANGE_DELAY;
        }

        if (t >= prev_t + EJECTOR_UP_DELAY && !step9Done && step8Done) {
            ejector.setPosition(ejectorDown);
            step9Done = true;
            prev_t += EJECTOR_UP_DELAY;
        }

        if (t >= prev_t + EJECTOR_DOWN_DELAY && !step10Done && step9Done) {
            Posspinner = 0.0;
            step10Done = true;

            // ===================== AUTO RESET TO INTAKE AFTER SHOOTING =====================
            outtakeMode = false;

            // Go straight back to intake without needing B/circle again
            intakeMode = true;
            spinIntake = true;
            intake.setPower(-1);

            Posspinner = 0.0;

            prev_t = 0;
            // ============================================================================
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // ================== INIT ==================
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        ejector = hardwareMap.get(Servo.class,"ejector");
        ejector.setDirection(Servo.Direction.REVERSE);
        // Motor coast for smoother spin
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Use encoder for velocity control
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply PIDF coefficients
        flywheel.setVelocityPIDFCoefficients(kP_v, kI_v, kD_v, kF_v);
        trajectoryAngleModifier = hardwareMap.get(Servo.class, "unghituretaoy");
       // trajectoryAngleModifier.setPosition(0.65);

        telemetry.addLine("Flywheel Full Hold Initialized");
        telemetry.update();
        ejector = hardwareMap.get(Servo.class, "ejector");
        spinnerFar = hardwareMap.get(Servo.class, "SpinnerFar");
        spinnerCLose = hardwareMap.get(Servo.class, "SpinnerClose");
        intake = hardwareMap.get(DcMotor.class, "intake");

        ejector.setPosition(ejectorDown);
        spinnerFar.setPosition(0);
        spinnerCLose.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {TARGET_TPS = TARGET_RPM * FLYWHEEL_TICKS_PER_REV / 60.0; // ticks/sec
            // ================== HOLD TARGET VELOCITY ==================
            flywheel.setVelocity(TARGET_TPS);

            if (gamepad1.yWasPressed())
            {
                TARGET_RPM+=25;
            }
            if (gamepad1.crossWasPressed())
            {
                TARGET_RPM-=25;
            }
            if (gamepad1.dpad_up)
            {
                ejector.setPosition(ejectorUp);
            }
            else ejector.setPosition(ejectorDown);
            if (gamepad1.squareWasPressed())
            {
                kP_v+=1;
            }
            else if (gamepad1.circleWasPressed())
            {
                kP_v-=1;
            }
            else if (gamepad1.leftBumperWasPressed())
            {
                kF_v+=1;
            }
            else if (gamepad1.rightBumperWasPressed())
            {
                kF_v-=1;
            }

            if (gamepad1.psWasPressed())runOuttake();
            // Telemetry to monitor RPM
            double currentRPM = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
            double appliedSpinnerPos = Posspinner + SPINDEXER_INTAKE_OFFSET;
            if (gamepad1.psWasPressed()) {
                outtakeMode = true;
                outtakeTimeout.reset();
                prev_t = 0;
            }
            if (outtakeMode) runOuttake();

            spinnerFar.setPosition(appliedSpinnerPos);
            spinnerCLose.setPosition(appliedSpinnerPos);

            if (gamepad1.dpadDownWasPressed())Posspinner+=0.19;
            if (gamepad1.shareWasPressed())intake.setPower(-1.0);
            if (gamepad1.optionsWasPressed())Posspinner=0.085;
            telemetry.addData("Flywheel RPM", currentRPM);
            telemetry.addData("Flywheel Target RPM", TARGET_RPM);
            telemetry.addData("kp", kP_v);
            telemetry.addData("kf", kF_v);
            telemetry.update();
        }
    }
}
