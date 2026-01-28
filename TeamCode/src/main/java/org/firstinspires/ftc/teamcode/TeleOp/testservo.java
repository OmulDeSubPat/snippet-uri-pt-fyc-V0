package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="testservo")
public class testservo extends LinearOpMode {

    private Servo ejectorServo;
    private double servoPosition = 0.0; // start position
    private final double increment = 0.0001; // slow movement step
    private final double minPos = 0.0;
    private final double maxPos = 1.0;

    private ElapsedTime time = new ElapsedTime();

    // PS button sequence variables
    private boolean psActive = false;
    private double psStartTime = 0;
    DcMotor flywheel;

    @Override
    public void runOpMode() {
        ejectorServo = hardwareMap.get(Servo.class, "ejector");
        flywheel = hardwareMap.get(DcMotor.class,"flywheel");

        // Reverse servo if needed
        ejectorServo.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            // --------------------
            // Slow D-pad movement
            // --------------------
            if (gamepad1.dpad_up) {
                servoPosition += increment; // move up slowly
            }
            if (gamepad1.dpad_down) {
                servoPosition -= increment; // move down slowly
            }

            // --------------------
            // Quick preset positions
            // --------------------
            if (gamepad1.yWasPressed()) {
                servoPosition = 0.18; // quick up
            }
            if (gamepad1.crossWasPressed()) {
                servoPosition = 0.02; // quick down
            }

            // --------------------
            // PS button 2-step sequence
            // --------------------
            if (gamepad1.psWasPressed() && !psActive) {
                servoPosition = 0.02; // step 1
                psActive = true;
                psStartTime = time.milliseconds();
            }

            // Check if 100 ms passed to do step 2
            if (psActive && time.milliseconds() - psStartTime >= 200) {
                servoPosition = 0.18; // step 2
                psActive = false;      // sequence done
            }

            // --------------------
            // Clamp servo position
            // --------------------
            servoPosition = Math.max(minPos, Math.min(maxPos, servoPosition));
            ejectorServo.setPosition(servoPosition);

            // --------------------
            // Telemetry
            // --------------------
            if (gamepad1.right_bumper)flywheel.setPower(0.4);
            if (gamepad1.left_bumper)flywheel.setPower(0);
            telemetry.addData("Servo Position", servoPosition);
            telemetry.addData("Direction", ejectorServo.getDirection());
            telemetry.addData("PS Sequence Active", psActive);
            telemetry.update();
        }
    }
}
