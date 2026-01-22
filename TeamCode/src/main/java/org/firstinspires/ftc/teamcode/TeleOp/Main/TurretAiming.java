package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class TurretAiming {
    DcMotorEx MotorTureta;//cel care misca tureta

    //Constante PID pt Tureta doar
    //TODO//sa se puna pid ul si pt spinner/rezolver
    double kP = 0.002;
    double kI = 0.0001;
    double kD = 0.0006;
    double integral = 0;
    double lastError = 0;
    long lastTime = 0;


    //Variabile pentru cautare AprilTag
    double scanSpeed = 0.05;
    boolean scanDir = true;
    double lastTx = 0;
    double txDeadzone = 3.5;//TX deadzone pt oprire tureta
    double nudgePower = 0.0; //puterea nudge ului pentru deblocarea turetei(nu cred ca o sa folosesc asa ceva~tibichi)


    //Variabile pentru durata de cautare aka nebunia lu tibi
    long UltimaDataVazut=0;
    long TimpDeLaPierdereaTargetului=0;
    double TimpPauza=0.3;
    double TimpCautareLocala=6;

    //variabile cautare locala(stanga-dreapta)
    double PutereScanareLocala=0.12;    //TO BE DETERMINED(empiric)
    double PerioadaSchimbariiSensului=0.4;  //TO BE DETERMINED(empiric)


    //Limitare tureta
    double LEFT_LIMIT = -100;
    double RIGHT_LIMIT = 100;

    double DEG_PER_TICK = 0;


    //Variabile Misc/Vision
    Limelight3A limelight;
    IMU imu;

    boolean ConditieScanarePlanetara =false;//cautare planeta inseamna
    // cautarea mai extinsa iar cautarea locala cea cu un range mai mic
    //pt prosti o trebuit sa scriu asta

    private final LinearOpMode opMode;

    public TurretAiming(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(orientation));

        MotorTureta = hardwareMap.get(DcMotorEx.class, "turret");
        MotorTureta.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        MotorTureta.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MotorTureta.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        MotorTureta.setPower(0);
    }

    public double Limitare(double Putere)
    {
        double UnghiTureta=MotorTureta.getCurrentPosition()*DEG_PER_TICK;
        if (UnghiTureta >= RIGHT_LIMIT) {
            integral = 0;
            if (Putere > 0) {
                // nudge = un mic "cot" in caz ca se blocheaza tureta
                return -nudgePower;
            }
        }

        if (UnghiTureta <= LEFT_LIMIT) {
            integral = 0;
            if (Putere < 0) {
                return nudgePower;
            }
        }
        return Putere;
    }

    public void runAiming() {
        while (opMode.opModeIsActive()) {
            //tibi schizo
            lastTime = System.nanoTime();
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
                    MotorTureta.setPower(0);
                    opMode.telemetry.addData("Status", "Pauza de cautare");
                    opMode.telemetry.addData("Vazut acum:", UltimaDataVazutSecunde);
                    opMode.telemetry.update();
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
                    MotorTureta.setPower(Putere);

                    opMode.telemetry.addData("Status", "Cautare locala");
                    opMode.telemetry.addData("Timpul local", TimpulLocal);
                    opMode.telemetry.addData("Ultimul tx", lastTx);
                    opMode.telemetry.update();
                }
                ConditieScanarePlanetara= UltimaDataVazutSecunde > TimpCautareLocala;

                if (ConditieScanarePlanetara == true)
                    scanDir = lastTx > 0;
                double UnghiTureta = MotorTureta.getCurrentPosition() * DEG_PER_TICK;
                double PutereCautare=scanSpeed*(scanDir ? 1 : -1);
                PutereCautare = Limitare(PutereCautare);
                MotorTureta.setPower(PutereCautare);

                opMode.telemetry.addData("Status:", "Cautare planetara");
                opMode.telemetry.addData("Unghi Tureta", UnghiTureta);
                opMode.telemetry.update();
            }
            // opresti cautarea daca gaseste tinta
            if (Math.abs(tx) <= txDeadzone) {
                MotorTureta.setPower(0);
                integral = 0;
                lastError = 0;
                ConditieScanarePlanetara=false;

                opMode.telemetry.addData("Aligned (TX in ±1.5)", true);
                opMode.telemetry.addData("tx", tx);
                opMode.telemetry.update();
            }

            // PID
            long now = System.nanoTime();
            double dt = (now - lastTime) / 1e9;
            lastTime = now;

            double error = -tx;
            integral += error * dt;
            double derivative = (error - lastError) / dt;
            lastError = error;

            double output = kP * error + kI * integral + kD * derivative;
            output = Range.clip(output, -0.5, 0.5);

            output = Limitare(output);

            MotorTureta.setPower(output);

            opMode.telemetry.addData("tx", tx);
            opMode.telemetry.addData("PID Output", output);
            opMode.telemetry.addData("Turret Angle", MotorTureta.getCurrentPosition() * DEG_PER_TICK);
            opMode.telemetry.addData("Scanning", false);
            opMode.telemetry.update();
        }
    }
}