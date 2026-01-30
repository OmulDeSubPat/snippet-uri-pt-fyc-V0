package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "FlywheelTest")
public class flywheeltest extends LinearOpMode {

    // ================== FLYWHEEL CONSTANTS ==================
    static final double FLYWHEEL_TICKS_PER_REV = 28.0;    // REV encoder ticks per rev
    static double TARGET_RPM = 3800;              // Desired flywheel speed
    static double TARGET_TPS = TARGET_RPM * FLYWHEEL_TICKS_PER_REV / 60.0; // ticks/sec

    // PIDF coefficients tuned for stable velocity
    static double kP_v = 10.0;
    static final double kI_v = 0.0;
    static final double kD_v = 0.0;
    static double kF_v = 13.0;
    Servo trajectoryAngleModifier;

    DcMotorEx flywheel;
    Servo ejector;

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
        trajectoryAngleModifier.setPosition(0.65);

        telemetry.addLine("Flywheel Full Hold Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {TARGET_TPS = TARGET_RPM * FLYWHEEL_TICKS_PER_REV / 60.0; // ticks/sec
            // ================== HOLD TARGET VELOCITY ==================
            flywheel.setVelocity(TARGET_TPS);

        /*    if (gamepad1.yWasPressed())
            {
                TARGET_RPM+=100;
            }
            if (gamepad1.crossWasPressed())
            {
                TARGET_RPM-=100;
            }*/
            if (gamepad1.dpad_up)
            {
                ejector.setPosition(0.02);
            }
            else ejector.setPosition(0.19);
            if (gamepad1.crossWasPressed())
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
            // Telemetry to monitor RPM
            double currentRPM = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
            telemetry.addData("Flywheel RPM", currentRPM);
            telemetry.addData("Flywheel Target RPM", TARGET_RPM);
            telemetry.addData("kp", kP_v);
            telemetry.addData("kf", kF_v);
            telemetry.update();
        }
    }
}
