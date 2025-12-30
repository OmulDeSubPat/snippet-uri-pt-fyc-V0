package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Turret {

    DcMotorEx MotorTureta;//cel care misca tureta

    //Constante PID
    double kP = 0.002;
    double kI = 0.0001;
    double kD = 0.0006;
    double integral = 0;
    double lastError = 0;
    long lastTime = 0;

    //Limitare tureta
    double LEFT_LIMIT = -100;
    double RIGHT_LIMIT = 100;

    double DEG_PER_TICK = 360.0 / 560.0; //Conversie grad/tick
    double nudgePower = 0.0; //puterea nudge ului

    LinearOpMode opMode;

    public Turret(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        MotorTureta = opMode.hardwareMap.get(DcMotorEx.class, "turret");
        MotorTureta.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        MotorTureta.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MotorTureta.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        MotorTureta.setPower(0);
    }

    public static double Limitare(double Putere) {
        double UnghiTureta = MotorTureta.getCurrentPosition() * DEG_PER_TICK;
        if (UnghiTureta >= RIGHT_LIMIT) {
            integral = 0;
            if (Putere > 0) return -nudgePower;
        }

        if (UnghiTureta <= LEFT_LIMIT) {
            integral = 0;
            if (Putere < 0) return nudgePower;
        }
        return Putere;
    }
}
