package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@TeleOp(name="TestareTruetaAim")
public class turetatest extends LinearOpMode {

    //Variabile pentru durata de cautare aka nebunia lu tibi
    long UltimaDataVazut=0;
    long TimpDeLaPierdereaTargetului=0;
    double TimpPauza=0.3;
    double TimpCautareLocala=6;

    //variabile cautare locala(stanga-dreapta)
    double PutereScanareLocala=0.18;    //TO BE DETERMINED(empiric)
    double PerioadaSchimbariiSensului=1;  //TO BE DETERMINED(empiric)
    DcMotorEx MotorTureta;
    Limelight3A limelight;
    IMU imu;

    DcMotor front_left, front_right, back_left, back_right;

    // PID constants + state
    double kP = 0.002, kI = 0.0001, kD = 0.0006;
    double integral = 0, lastError = 0;
    long lastTime = 0;

    boolean ConditieScanarePlanetara =false;//cautare planeta inseamna
    // cautarea mai extinsa iar cautarea locala cea cu un range mai mic
    //pt prosti o trebuit sa scriu asta
    // scanning vars (keep your existing ones)
    double scanSpeed = 0.05;
    boolean scanDir = true;
    double lastTx = 0;
    double txDeadzone = 3.5;

    double LEFT_LIMIT = -100, RIGHT_LIMIT = 100;
    double DEG_PER_TICK = 360.0 / 560.0;

    @Override
    public void runOpMode() {
        InitWheels();

        // Initializare limelight, imu, motor tureta
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(orientation));

        MotorTureta = hardwareMap.get(DcMotorEx.class, "tureta");
        MotorTureta.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        MotorTureta.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MotorTureta.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        MotorTureta.setPower(0);

        //DcMotorEx flywheel;
        //flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        boolean AimOn=false; //state ul in care este tureta:fals daca nu cauta, true daca da


        waitForStart();

        lastTime = System.nanoTime();
        while (opModeIsActive()) {
            SetWheelsPower();
            runAiming();
            if(gamepad1.dpad_down){
                if(!AimOn){
                    runAiming();
                    AimOn=true;
                }
                else if (gamepad1.ps){
                    AimOn=false;
                }
            }
            if(gamepad1.circleWasPressed()){
               // flywheel.setPower(1);
                //adaugare ejector
            }
            telemetry.update();
            idle();
        }
    }

    private void InitWheels() {
        front_left = hardwareMap.dcMotor.get("lf");
        front_right = hardwareMap.dcMotor.get("rf");
        back_left = hardwareMap.dcMotor.get("lr");
        back_right = hardwareMap.dcMotor.get("rr");

        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void SetWheelsPower() {
        double left_x = gamepad1.left_stick_x;
        double left_y = -gamepad1.left_stick_y;
        double right_x = gamepad1.right_stick_x;

        double fl = left_y + left_x + right_x;
        double bl = left_y - left_x + right_x;
        double fr = left_y - left_x - right_x;
        double br = left_y + left_x - right_x;

        double max = Math.max(Math.abs(fl),
                Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br))));
        if (max > 1.0) {
            fl /= max; bl /= max; fr /= max; br /= max;
        }

        // IMPORTANT: actually set motor power
        front_left.setPower(fl);
        back_left.setPower(bl);
        front_right.setPower(fr);
        back_right.setPower(br);
    }

    public void runAiming() {

        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(ypr.getYaw());

        LLResult result = limelight.getLatestResult();
        double tx = 0;

        long OraActuala = System.nanoTime();

        double dt = (OraActuala - lastTime) / 1e9;
        lastTime = OraActuala;
        if (dt <= 0) return;

        //cand targetul este in raza de actiune
        if (result != null && result.isValid()) {

            tx = -result.getTx();
            lastTx = tx;
            UltimaDataVazut = OraActuala;
            TimpDeLaPierdereaTargetului = 0;

            // Deadzone
            if (Math.abs(tx) <= txDeadzone) {
                MotorTureta.setPower(0);
                integral = 0;
                lastError = 0;
                ConditieScanarePlanetara = false;
                return;
            }

            // PID
            double error = -tx;
            integral += error * dt;
            double derivative = (error - lastError) / dt;
            lastError = error;

            double output = kP * error + kI * integral + kD * derivative;
            output = Range.clip(output, -0.5, 0.5);
            output = Limitare(output);

            MotorTureta.setPower(output);
            return;
        }

        //Cand tagetul incepe sa fie pierdut
        double UltimaDataVazutSecunde = (OraActuala - UltimaDataVazut) / 1e9;

        // Pauza
        if (UltimaDataVazutSecunde <= TimpPauza) {
            MotorTureta.setPower(0);
            return;
        }

        // Initializeaza timpul de cautare
        if (TimpDeLaPierdereaTargetului == 0)
            TimpDeLaPierdereaTargetului = OraActuala;

        // Cautare locala
        if (UltimaDataVazutSecunde <= TimpCautareLocala) {

            double TimpulLocal =
                    (OraActuala - TimpDeLaPierdereaTargetului) / 1e9;

            double DirectiaInitiala = (lastTx > 0) ? 1.0 : -1.0;

            boolean SchimbareSens =
                    ((int)(TimpulLocal / PerioadaSchimbariiSensului)) % 2 == 0;

            double Directie = SchimbareSens
                    ? DirectiaInitiala
                    : -DirectiaInitiala;

            double Putere =
                    Limitare(PutereScanareLocala * -Directie);

            MotorTureta.setPower(Putere);
            return;
        }

        // Cautare planetara
        ConditieScanarePlanetara = true;
        scanDir = lastTx > 0;

        double PutereCautare =
                Limitare(scanSpeed * (scanDir ? 1 : -1));

        MotorTureta.setPower(PutereCautare);
    }

    private double Limitare(double power) {
        double angleDeg = MotorTureta.getCurrentPosition() * DEG_PER_TICK;

        if (angleDeg >= RIGHT_LIMIT && power > 0) return 0;
        if (angleDeg <= LEFT_LIMIT && power < 0) return 0;

        return power;
    }
    private double Unghi(){
        return 1;
    }//TO DO:calculeaza unghiul optim pt lansare
}
