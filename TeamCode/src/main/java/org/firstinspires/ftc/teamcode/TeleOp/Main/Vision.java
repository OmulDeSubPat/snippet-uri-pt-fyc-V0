package org.firstinspires.ftc.teamcode.TeleOp.Main;

import static org.firstinspires.ftc.teamcode.TeleOp.Main.Turret.Limitare;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.util.Range;

public class Vision {

    Limelight3A limelight;
    IMU imu;
    Turret turret;
    LinearOpMode opMode;

    double scanSpeed = 0.05;
    boolean scanDir = true;
    double lastTx = 0;
    double txDeadzone = 3.5;

    long UltimaDataVazut = 0;
    long TimpDeLaPierdereaTargetului = 0;
    double TimpPauza = 0.3;
    double TimpCautareLocala = 6;

    double PutereScanareLocala = 0.12;
    double PerioadaSchimbariiSensului = 0.4;

    boolean ConditieScanarePlanetara = false;
    double kP = 0.002;
    double kI = 0.0001;
    double kD = 0.0006;
    double integral = 0;
    double lastError = 0;
    long lastTime = 0;

    public Vision(LinearOpMode opMode, Turret turret) {
        this.opMode = opMode;
        this.turret = turret;
    }

    public void limeInit() {
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start();

        imu = opMode.hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(orientation));
    }

    public void aliniereTureta()
    {
        long lastTime = System.nanoTime();
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(ypr.getYaw());

        LLResult result = limelight.getLatestResult();
        double tx=0;
        long OraActuala=System.nanoTime(); //start timer pentru cazul in care nu mai e target
        if (result != null && result.isValid()) {
            tx = -result.getTx();
            lastTx = tx;
            UltimaDataVazut=OraActuala;
            TimpDeLaPierdereaTargetului=0;
        } else {
            double UltimaDataVazutSecunde = (OraActuala - UltimaDataVazut) / 1e9;
            if (UltimaDataVazutSecunde <= TimpPauza) {
             //   MotorTureta.setPower(0);
            }
            if (TimpDeLaPierdereaTargetului == 0)
                TimpDeLaPierdereaTargetului = OraActuala;


            if (UltimaDataVazutSecunde <= TimpCautareLocala  && UltimaDataVazutSecunde >=TimpPauza) {
                double TimpulLocal = (OraActuala - TimpDeLaPierdereaTargetului) / 1e9; //timpul de cand a inceput cautarea locala
                double DirectiaInitiala;
                if (lastTx > 0)
                    DirectiaInitiala = 1.0;
                else
                    DirectiaInitiala = -1.0;


                boolean SchimbareSens;
                if ((int) (TimpulLocal / PerioadaSchimbariiSensului) % 2 == 0)
                    SchimbareSens = true;
                else
                    SchimbareSens = false;


                double Directie;
                if (SchimbareSens)
                    Directie = DirectiaInitiala;
                else
                    Directie = -DirectiaInitiala;


                double Putere = Limitare(PutereScanareLocala * -Directie);
                //MotorTureta.setPower(Putere);

            }
            ConditieScanarePlanetara= UltimaDataVazutSecunde > TimpCautareLocala;

            if (ConditieScanarePlanetara == true)
                scanDir = lastTx > 0;
        //    double UnghiTureta = MotorTureta.getCurrentPosition() * DEG_PER_TICK;
            double PutereCautare=scanSpeed*(scanDir ? 1 : -1);
            PutereCautare = Limitare(PutereCautare);
          //  MotorTureta.setPower(PutereCautare);

        }
        // STOP MOTOR IF TX IS ±1.5
        if (Math.abs(tx) <= txDeadzone) {
         //   MotorTureta.setPower(0);
            integral = 0;
            lastError = 0;
            ConditieScanarePlanetara=false;

        }

        // PID CONTROL
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        double error = -tx;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = kP * error + kI * integral + kD * derivative;
        output = Range.clip(output, -0.5, 0.5);

        // Apply limits with nudge if stuck
        output = Limitare(output);

       // MotorTureta.setPower(output);


    }
    }
