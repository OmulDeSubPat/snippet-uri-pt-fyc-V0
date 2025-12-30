package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name="TeleOpServosAndPivoter")
public class TeleOpMain extends LinearOpMode {

    //Motor sasiu
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;

    // Motoare unrelated de sasiu

    Servo ejector ;//servo ul care baga mingile in flywheel
    DcMotor matura;//intake ul activ, (ala pasiv unde e? ~tibichi)
    DcMotor flywheel;//cel care lanseaza mingea
    DcMotor spinner;//cel care roteste mingile in rezervor
    DcMotorEx MotorTureta;//cel care misca tureta

    //Constante PID
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


    //Variabile pentru durata de cautare
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

    double DEG_PER_TICK = 360.0 / 560.0; //Conversie grad/tick


    //variabile colorate 😉
    int[] last5 = new int[5];
    int index = 0;
    ColorSensor colorsensor;

    // Variabile Misc
    Limelight3A limelight;
    IMU imu;

    boolean ConditieScanarePlanetara =false;

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
    public void InitColorSensor() {
        colorsensor = hardwareMap.colorSensor.get("colorsensor");
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
        ejector = hardwareMap.servo.get("ejector");

    }

    private void pozInit() {
//nuj daca o sa existe
    }


    private void DcInit() {
        MotorTureta = (DcMotorEx) hardwareMap.dcMotor.get("tureta");
        MotorTureta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorTureta.setTargetPosition(0);
        MotorTureta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorTureta.setPower(0);

        matura = hardwareMap.dcMotor.get("matura");

        flywheel = hardwareMap.dcMotor.get("flywheel");
        flywheel.setPower(0.3);

        spinner = hardwareMap.dcMotor.get("spinner");
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setTargetPosition(0);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(0);
    }


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
                // nudge slightly to the left
                return -nudgePower;
            }
        }

        if (UnghiTureta <= LEFT_LIMIT) {
            integral = 0;
            if (Putere < 0) {
                // nudge slightly to the right
                return nudgePower;
            }
        }
        return Putere;
    }

    public void pozitii()
    {
        //intake
        if (gamepad2.circleWasPressed())
        {
            matura.setPower(1);
            flywheel.setPower(0.3);
        }

        //outtake
        if (gamepad2.xWasPressed())
        {
            flywheel.setPower(1);
            matura.setPower(0);
            ejector.setPosition(0.555);
            //sa se miste 120 de grade spinner ul

        }
    }

    public void aliniereTureta()
    {
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
                    telemetry.addData("Status", "Pauza de cautare");
                    telemetry.addData("Vazut acum:", UltimaDataVazutSecunde);
                    telemetry.update();
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

                    telemetry.addData("Status", "Cautare locala");
                    telemetry.addData("Timpul local", TimpulLocal);
                    telemetry.addData("Ultimul tx", lastTx);
                    telemetry.update();
                }
                ConditieScanarePlanetara= UltimaDataVazutSecunde > TimpCautareLocala;

                if (ConditieScanarePlanetara == true)
                    scanDir = lastTx > 0;
                double UnghiTureta = MotorTureta.getCurrentPosition() * DEG_PER_TICK;
                double PutereCautare=scanSpeed*(scanDir ? 1 : -1);
                PutereCautare = Limitare(PutereCautare);
                MotorTureta.setPower(PutereCautare);

                telemetry.addData("Status:", "Cautare planetara");
                telemetry.addData("Unghi Tureta", UnghiTureta);
                telemetry.update();
            }
            // STOP MOTOR IF TX IS ±1.5
            if (Math.abs(tx) <= txDeadzone) {
                MotorTureta.setPower(0);
                integral = 0;
                lastError = 0;
                ConditieScanarePlanetara=false;

                telemetry.addData("Aligned (TX in ±1.5)", true);
                telemetry.addData("tx", tx);
                telemetry.update();
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

            MotorTureta.setPower(output);

            telemetry.addData("tx", tx);
            telemetry.addData("PID Output", output);
            telemetry.addData("Turret Angle", MotorTureta.getCurrentPosition() * DEG_PER_TICK);
            telemetry.addData("Scanning", false);
            telemetry.update();

    }
    private double getHue(int r, int g, int b) {
        int max = Math.max(r, Math.max(g, b));
        int min = Math.min(r, Math.min(g, b));
        if (max == min) return 0.0;
        double chroma = max - min;
        double h;
        if (max == r) h = (double)(g - b) / chroma;
        else if (max == g) h = (double)(b - r) / chroma + 2.0;
        else h = (double)(r - g) / chroma + 4.0;
        h *= 60.0;
        if (h < 0) h += 360.0;
        return h;
    }
    private int processColorSensor() {
        int r = colorsensor.red();
        int g = colorsensor.green();
        int b = colorsensor.blue();
        int alpha = colorsensor.alpha();

        double h = getHue(r, g, b);

        int detected;
        if (alpha < 100 && (h == 150 || h == 144)) detected = 0;
        else if ((h > 215) || (alpha < 100 && (h == 160 || h == 180))) detected = 2;
        else if (h > 135 && h < 160) detected = 1;
        else if ((h == 210 || h == 220 || h == 225 || h == 200) && alpha < 100) detected = 2;
        else detected = 0;

        last5[index] = detected;
        index = (index + 1) % 5;

        int count0 = 0, count1 = 0, count2 = 0;
        for (int v : last5) {
            if (v == 0) count0++;
            else if (v == 1) count1++;
            else if (v == 2) count2++;
        }

        int finalColor = 0;
        if (count1 >= 3 && count1 > count2) finalColor = 1;
        else if (count2 >= 3 && count2 > count1) finalColor = 2;

        /*telemetry.addData("R", r);
        telemetry.addData("G", g);
        telemetry.addData("B", b);
        telemetry.addData("Alpha", alpha);
        telemetry.addData("Hue", String.format(Locale.US, "%.1f", h));
        telemetry.addData("Detected", detected);*/
        telemetry.addData("Final Color", finalColor);
        telemetry.update();

        return finalColor;
    }

    @Override
    public void runOpMode() {
        servoInit();
        DcInit();
        pozInit();
        InitWheels();
        limeInit();


        waitForStart();

        Thread wheelThread = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                SetWheelsPower();
            }
        });

        Thread systemsThread = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                pozitii();
                UpdateTelemetry();

            }
        });

        Thread AimThread = new Thread(() ->{
            while(opModeIsActive() && !isStopRequested())
                aliniereTureta();
        });

        wheelThread.start();
        systemsThread.start();
        AimThread.start();

        while (opModeIsActive() && !isStopRequested()) {
            idle();
        }
    }

    int[] slots = new int[3];
    int slotIndex = 0;

    private int[] updateColorVector() {
        int color = processColorSensor();

        slots[slotIndex] = color;
        slotIndex = (slotIndex + 1) % 3;

        return slots;
    }

    private void resetSlots() {
        slots[0] = 0;
        slots[1] = 0;
        slots[2] = 0;
        slotIndex = 0;
    }
    private void UpdateTelemetry() {

        telemetry.update();
    }
}