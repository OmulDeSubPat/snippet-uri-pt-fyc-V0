package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="SpinnerPIDControl")
public class PIDtunner extends LinearOpMode {

    DcMotorEx spinner;

    // Encoder constants
    static final double TICKS_PER_REV = 384.5;
    static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;

    // PID coefficients (P is tunable)
    double P = 0.008;
    double I = 0.0000;
    double D = 0.003;

    double integralSum = 0;
    double lastError = 0;


    boolean xPrev = false;

    @Override
    public void runOpMode() {

        spinner = hardwareMap.get(DcMotorEx.class, "Spinner");

        spinner.setDirection(DcMotorSimple.Direction.FORWARD);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        int targetTicks = 0;

        while (opModeIsActive()) {

            // Target positions
            if (gamepad1.aWasPressed()) {
                targetTicks = (int)(60 * TICKS_PER_DEGREE);
            }
            if (gamepad1.bWasPressed()) {
                targetTicks = 0;
            }
            if (gamepad1.xWasPressed()) {
                targetTicks = (int)(120 * TICKS_PER_DEGREE);
            }

            /* =========================
               REAL-TIME P TUNING
               ========================= */

            // D-pad UP → increase P
            if (gamepad1.dpadUpWasPressed() ) {
                P += 0.0005;
            }

            // D-pad DOWN → decrease P
            if (gamepad1.dpadDownWasPressed() ) {
                P -= 0.0005;
            }

            // D-pad UP → increase P
            if (gamepad1.dpadLeftWasPressed() ) {
                D += 0.0001;
            }

            // D-pad DOWN → decrease P
            if (gamepad1.dpadRightWasPressed() ) {
                D -= 0.0001;
            }




            /* =========================
               PID CONTROL
               ========================= */

            double currentPos = spinner.getCurrentPosition();
            double error = targetTicks - currentPos;

            integralSum += error;
            double derivative = error - lastError;
            lastError = error;

            double pidOutput = error * P + integralSum * I + derivative * D;
            pidOutput = Math.max(-1, Math.min(1, pidOutput));

            spinner.setPower(pidOutput);

            /* =========================
               TELEMETRY
               ========================= */

            telemetry.addData("Target (deg)", targetTicks / TICKS_PER_DEGREE);
            telemetry.addData("Current (deg)", currentPos / TICKS_PER_DEGREE);
            telemetry.addData("Error", error);
            telemetry.addData("P", P);
            telemetry.addData("D", D);
            telemetry.addData("PID Power", pidOutput);
            telemetry.addLine("D-pad Up/Down: Tune P | X: Reset I");
            telemetry.update();
        }
    }
}
