package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="TurretAim_StopAtTX_±1.5_WithNudge")
public class limelight extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;
    private DcMotorEx turretMotor;

    // PID constants
    double kP = 0.002;
    double kI = 0.0001;
    double kD = 0.0006;

    double integral = 0;
    double lastError = 0;
    long lastTime = 0;

    // Scanning
    double scanSpeed = 0.05;
    boolean scanDir = true;

    double DEG_PER_TICK = 360.0 / 560.0;

    // Turret hard limits (degrees)
    double LEFT_LIMIT = -100;
    double RIGHT_LIMIT = 100;

    double lastTx = 0;

    // TX deadzone for stopping turret
    double txDeadzone = 3.5;

    // Nudge power to get turret unstuck
    double nudgePower = 0.0;

    // variabile relativiste(timpice)
    long UltimaDataVazut=0;
    long TimpDeLaPierdereaTargetului=0;
    double TimpPauza=0.3;
    double TimpCautareLocala=6;

    //variabile pt cautarea locala
    double PutereScanareLocala=0.12;    //TO BE CONTINUED(cred ca impiric)
    double PerioadaSchimbariiSensului=0.4;  //TO BE CONTINUED(cred ca impiric)


    //variabila pt cautarea planetara
    boolean ConditieScanarePlanetara=false;


    // -------------------------
    // INIT
    // -------------------------
    public void limeInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start(); // REQUIRED

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(orientation));

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turretMotor.setPower(0);
    }

    // -------------------------
    // APPLY TURRET HARD LIMITS WITH NUDGE
    // -------------------------
    private double applyLimits(double desiredPower) {
        double turretAngle = turretMotor.getCurrentPosition() * DEG_PER_TICK;

        if (turretAngle >= RIGHT_LIMIT) {
            integral = 0;
            if (desiredPower > 0) {
                // nudge slightly to the left
                return -nudgePower;
            }
        }

        if (turretAngle <= LEFT_LIMIT) {
            integral = 0;
            if (desiredPower < 0) {
                // nudge slightly to the right
                return nudgePower;
            }
        }

        return desiredPower;
    }

    // -------------------------
    // MAIN LOOP
    // -------------------------
    @Override
    public void runOpMode() {
        limeInit();
        waitForStart();

        lastTime = System.nanoTime();

        while (opModeIsActive()) {
            YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(ypr.getYaw());

            LLResult result = limelight.getLatestResult();
            double tx;
            long OraActuala=System.nanoTime(); //start timer pentru cazul in care nu mai e target
            if (result != null && result.isValid()) {
                tx = -result.getTx();
                lastTx = tx;
                UltimaDataVazut=OraActuala;
            } else {
                double UltimaDataVazutSecunde = (OraActuala - UltimaDataVazut) / 1e9;
                if (UltimaDataVazutSecunde <= TimpPauza) {
                    turretMotor.setPower(0);
                    telemetry.addData("Status", "Pauza de cautare");
                    telemetry.addData("Vazut acum:", UltimaDataVazutSecunde);
                    telemetry.update();
                    continue;
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


                    double Putere = applyLimits(PutereScanareLocala * -Directie);
                    turretMotor.setPower(Putere);

                    telemetry.addData("Status", "Cautare locala");
                    telemetry.addData("Timpul local", TimpulLocal);
                    telemetry.addData("Ultimul tx", lastTx);
                    telemetry.update();
                    continue;
                }
                if (ConditieScanarePlanetara == true) {
                    if (lastTx > 0)
                        scanDir = true;
                    else
                        scanDir = false;
                    ConditieScanarePlanetara = true;
                }
                double UnghiTureta = turretMotor.getCurrentPosition() * DEG_PER_TICK;
                if (UnghiTureta >= RIGHT_LIMIT)
                    scanDir = false;
                else if (UnghiTureta <= LEFT_LIMIT)
                    scanDir = true;
                double PutereCautare=scanSpeed*(scanDir ? 1 : -1);
                PutereCautare = applyLimits(PutereCautare);
                turretMotor.setPower(PutereCautare);

                telemetry.addData("Status:", "Cautare planetara");
                telemetry.addData("Unghi Tureta", UnghiTureta);
                telemetry.update();
                continue;
            }
            // STOP MOTOR IF TX IS ±1.5
            if (Math.abs(tx) <= txDeadzone) {
                turretMotor.setPower(0);
                integral = 0;
                lastError = 0;
                ConditieScanarePlanetara=false;

                telemetry.addData("Aligned (TX in ±1.5)", true);
                telemetry.addData("tx", tx);
                telemetry.update();
                continue;
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
            output = applyLimits(output);

            turretMotor.setPower(output);

            telemetry.addData("tx", tx);
            telemetry.addData("PID Output", output);
            telemetry.addData("Turret Angle", turretMotor.getCurrentPosition() * DEG_PER_TICK);
            telemetry.addData("Scanning", false);
            telemetry.update();
        }
    }
}