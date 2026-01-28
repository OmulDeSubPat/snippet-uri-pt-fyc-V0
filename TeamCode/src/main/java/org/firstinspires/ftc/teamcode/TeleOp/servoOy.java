package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Trajectory Angle Servo Test")
public class servoOy extends LinearOpMode {

    Servo trajectoryAngleServo;

    // ===== CONFIG =====
    final double minAngle = 22.5;   // degrees
    final double maxAngle = 45.0;   // degrees

    final double gearRatio = 112.0 / 24.0;
    final double maxTravelDegrees = 300.0;

    final double posPerDegree = gearRatio / maxTravelDegrees;

    double currentAngle = minAngle;

    @Override
    public void runOpMode() {

        trajectoryAngleServo = hardwareMap.get(Servo.class, "unghituretaoy");

        setAngle(currentAngle);

        waitForStart();

        while (opModeIsActive()) {

            // Increase angle
            if (gamepad1.dpad_up) {
                currentAngle = maxAngle;
            }

            // Decrease angle
            if (gamepad1.dpad_down) {
                currentAngle =minAngle;
            }

            currentAngle = Range.clip(currentAngle, minAngle, maxAngle);
            setAngle(currentAngle);

            telemetry.addData("Trajectory Angle (deg)", currentAngle);
            telemetry.addData("Servo Position", trajectoryAngleServo.getPosition());
            telemetry.update();

            sleep(20);
        }
    }

    private void setAngle(double angle) {
        angle = Range.clip(angle, minAngle, maxAngle);
        double position = (angle - minAngle) * posPerDegree;
        trajectoryAngleServo.setPosition(position);
    }
}
