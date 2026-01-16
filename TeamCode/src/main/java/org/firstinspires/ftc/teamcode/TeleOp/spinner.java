package org.firstinspires.ftc.teamcode.TeleOp;

import android.text.method.Touch;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Arrays;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@TeleOp(name = "ColorDetection+cevaspinner")
public class spinner extends LinearOpMode {

    // Timers
    ElapsedTime pidTimer = new ElapsedTime();
    ElapsedTime outtakeTimer = new ElapsedTime();
    private ElapsedTime sortTimer = new ElapsedTime();

    // Color sensor memory
    int[] last5Sensor1 = new int[10];
    int indexSensor1 = 0;

    int encoderOffset=0;
    // Spinner slots
    int[] slots = new int[3];
    int[] slots2 = new int[3];
    int[] totem = { 2, 1, 2};
    int i=0;

    // Color tracking
    int lastStableColorSensor1 = 0;
    int lastStableColorSensor2 = 0;
    int lastStableColorSensor3 = 0;

    // Spinner state
    boolean spinnerBusy = false;
    boolean sortingActive = false;
    int lastSpinnerStep = 0;

    // Hardware
    ColorSensor colorsensorSLot1;
    // ColorSensor colorsensorSLot2;
    // ColorSensor colorsensorSLot3;
    DcMotorEx spinner;
    DcMotor intake;
    DcMotorEx tureta;
    Servo ejector;
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;
    DcMotor flywheel;

    // Spinner PID
    static final double TICKS_PER_REV = 384.5;
    static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;
    double P = 0.0101;
    double I = 0.0000;
    double D = 0.0015;
    double integralSum = 0;
    boolean wasFull = false;

    double lastError = 0;
    double targetTicks = 0;

    // Color detection timing
    long colorStartTime = 0;
    boolean colorPending = false;
    boolean spinnerMoving = false;
    boolean detectionLocked = false;
    boolean waitingForClear = false;
    boolean ThirdSlot = false;
    long UltimaDataVazut=0;
    long TimpDeLaPierdereaTargetului=0;
    double TimpPauza=0.3;
    double TimpCautareLocala=6;

    //variabile cautare locala(stanga-dreapta)
    double PutereScanareLocala=0.18;    //TO BE DETERMINED(empiric)
    double PerioadaSchimbariiSensului=1;  //TO BE DETERMINED(empiric)
    Limelight3A limelight;
    IMU imu;
    double kP = 0.0076, kI = 0.0001, kD = 0.0005;//pid tureta
    double integral = 0, lastErrorT = 0;
    long lastTime = 0;


    boolean ConditieScanarePlanetara =false;//cautare planeta inseamna
    // cautarea mai extinsa iar cautarea locala cea cu un range mai mic
    //pt prosti o trebuit sa scriu asta
    // scanning vars (keep your existing ones)
    double scanSpeed = 0.3;
    boolean scanDir = true;
    double lastTx = 0;
    double txDeadzone = 3.5;

    double DEG_PER_TICK = 360.0 / 560.0;
    boolean aVazutVreodataTarget = false;
    boolean outtake=false;
    boolean sorted=false;
    boolean intakeM=false;
    boolean flywheelOn = false;
    double flywheelInput = 1.0;   // pretend "full speed"
    boolean finalMoveDone = false;
    double DistantaPerete=0;
    double UnghiLansat=0;
    boolean activtureta=false;
    int mingi=0;
    long lastShotTime = 0;
    double turretFreezeAfterShot = 0.6; // seconds
    boolean ejecting = false;
    ElapsedTime ejectTimer = new ElapsedTime();
    boolean Empty=false;
  //  DigitalChannel limit;
    Boolean Home=false;
    TouchSensor limitswitch;
    boolean reset=false;

    int ballsToOuttake = 3;   // or however many
    int ballsOuttaken = 0;
    // Outtake simple control
    boolean outtakeActive = false;
    boolean ejectOpen = false;
    boolean ejectClose = false;
    boolean rotating = false;

    ElapsedTime outtakeStepTimer = new ElapsedTime();
    int atingeriSenzor=0;






    private double logFlywheel(double x) {
        // Tunable constants
        double a = 0.55;   // max power
        double b = 4.0;    // curve aggressiveness

        return a * Math.log(b * x + 1) / Math.log(b + 1);
    }



    private void simpleOuttake() {

        if (!outtakeActive) return;
        // --------------------
        // DONE
        // --------------------
        if (ballsOuttaken >= ballsToOuttake) {
            outtakeActive = false;
            return;
        }

        // --------------------
        // STEP 1: OPEN EJECTOR
        // --------------------
        if (!ejectOpen && !ejectClose && !rotating) {
            ejector.setPosition(0.005);
            outtakeStepTimer.reset();
            ejectOpen = true;
            return;
        }

        // --------------------
        // STEP 2: CLOSE EJECTOR
        // --------------------
        if (ejectOpen && outtakeStepTimer.seconds() >= 1.0) {
            ejector.setPosition(0.285);
            outtakeStepTimer.reset();
            ejectOpen = false;
            ejectClose = true;
            return;
        }

        // --------------------
        // STEP 3: ROTATE SPINNER
        // --------------------
        if (ejectClose && outtakeStepTimer.seconds() >= 1.0 && !spinnerMoving) {
            targetTicks += (120 * TICKS_PER_DEGREE);
            rotateSlotsRight();
            spinnerMoving = true;

            ballsOuttaken++;

            ejectClose = false;
            rotating = true;
            return;
        }

        // --------------------
        // STEP 4: WAIT FOR ROTATION
        // --------------------
        if (rotating && outtakeStepTimer.seconds() > 1.5) {
            spinnerMoving = false;
            rotating = false;
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

    private int getSpinnerStep() {
        int stepTicks = (int) (120 * TICKS_PER_DEGREE);
        return Math.round((float) getSpinnerPositionCorrected() / stepTicks);
    }


    private void SetWheelsPower() {
        double left_x = gamepad2.left_stick_x;
        double left_y = -gamepad2.left_stick_y; // forward is negative
        double right_x = gamepad2.right_stick_x;

        double front_left_pw = left_y + left_x + right_x;
        double back_left_pw = left_y - left_x + right_x;
        double front_right_pw = left_y - left_x - right_x;
        double back_right_pw = left_y + left_x - right_x;

        // Normalize so no motor power exceeds 1.0
        double max = Math.max(Math.abs(front_left_pw),
                Math.max(Math.abs(back_left_pw),
                        Math.max(Math.abs(front_right_pw), Math.abs(back_right_pw))));
        if (max > 1.0) {
            front_left_pw /= max;
            back_left_pw /= max;
            front_right_pw /= max;
            back_right_pw /= max;
        }

        front_left.setPower(front_left_pw);
        back_left.setPower(back_left_pw);
        front_right.setPower(front_right_pw);
        back_right.setPower(back_right_pw);
    }

    private double getHue(int r, int g, int b) {
        int max = Math.max(r, Math.max(g, b));
        int min = Math.min(r, Math.min(g, b));
        if (max == min) return 0.0;

        double chroma = max - min;
        double h;

        if (max == r) h = (double) (g - b) / chroma;
        else if (max == g) h = (double) (b - r) / chroma + 2.0;
        else h = (double) (r - g) / chroma + 4.0;

        h *= 60.0;
        if (h < 0) h += 360.0;

        return h;
    }

    private void CalibrareSpinner() {

        int ticksActual = getSpinnerPositionCorrected();

        double pasTicks = TICKS_PER_REV / 6.0; // 60°

        int celMaiApropiat =
                (int) Math.round(ticksActual / pasTicks) * (int) Math.round(pasTicks);

        int delta = celMaiApropiat - ticksActual;

        // corectie LOGICA permanenta
        encoderOffset += delta;

        // mentine consistenta PID-ului
        targetTicks += delta;
    }
    private int smekerie(ColorSensor colorSensor) {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        int alpha = colorSensor.alpha();

        double h = getHue(r, g, b);
        int detected;

        if (alpha<100 && (h==150 || h==144) ) detected = 0;
        else if ((h > 215) || (alpha<100 && (h==160 || h==180))) detected = 2;
        //else if (h > 135 && h < 160 && alpha>100) detected = 1;
        else if((h==140 || h==145) &&  alpha==43) detected = 0;
        else if (h > 135 && h < 160 && alpha>60)detected =1;
        else if ((h==210 || h==220 || h==225 || h==200) && alpha<100) detected=2;
        else detected = 0;

        return detected;
    }

    private int processSingleSensor(ColorSensor sensor, int[] last5, int index) {
        int detected = smekerie(sensor);
        last5[index] = detected;
        index = (index + 1) % 10;

        int count1 = 0; // green
        int count2 = 0; // purple

        for (int v : last5) {
            if (v == 1) count1++;
            else if (v == 2) count2++;
        }

        int finalColor = 0;
        if (count1 >= 6 && count1 > count2) finalColor = 1;
        else if (count2 >= 6 && count2 > count1) finalColor = 2;

        return finalColor;
    }


    private void updateAllSlots() {
        slots[0] = processSingleSensor(colorsensorSLot1, last5Sensor1, indexSensor1);
        indexSensor1 = (indexSensor1 + 1) % 10;
    }

    private boolean spinnerIsFull() {
        return slots2[0] != 0 && slots2[1] != 0 && slots2[2] != 0;
    }


    private void colorDrivenSpinnerLogic() {
        // Block NEW detections only
        if (!colorPending && (spinnerMoving || detectionLocked  ))
            return;
        if (waitingForClear) return;


        boolean newColorDetected = false;
        if (slots[0] != 0 && slots[0] != lastStableColorSensor1) {
            newColorDetected = true;
            lastStableColorSensor1 = slots[0];
        }

        if (newColorDetected && !spinner.isBusy() && !colorPending) {
            colorStartTime = System.currentTimeMillis();
            colorPending = true;
        }
        if (colorPending && System.currentTimeMillis() - colorStartTime >= 10) {
            targetTicks +=  (120 * TICKS_PER_DEGREE);
            spinnerMoving = true;
            detectionLocked = true;
            colorPending = false;

            // Always store new color in front slot
            slots2[0] = slots[0];

            // Rotate to match physical spinner movement
            rotateSlotsRight();

            slots[0] = 0;
            waitingForClear = true;   // 🔒 SAME BALL MUST CLEAR
        }

    }

    private void rotateSlotsRight() {
        int temp = slots2[2];
        slots2[2] = slots2[1];
        slots2[1] = slots2[0];
        slots2[0] = temp;
    }

    private void rotateSlotsLeft() {
        int temp = slots2[0];
        slots2[0] = slots2[1];
        slots2[1] = slots2[2];
        slots2[2] = temp;
    }


    public void runAiming() {
        if (!activtureta)return;
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(ypr.getYaw());

        LLResult result = limelight.getLatestResult();

        long OraActuala = System.nanoTime();

        double dt = (OraActuala - lastTime) / 1e9;
        lastTime = OraActuala;

        if (dt <= 0) return;
        if (dt < 0.005) dt = 0.005;

        // =========================
        // 1) TARGET VALID -> PID
        // =========================
        if (result != null && result.isValid()) {
            aVazutVreodataTarget = true;

            double tx = result.getTx();
            lastTx = tx;

            UltimaDataVazut = OraActuala;
            TimpDeLaPierdereaTargetului = 0;
            ConditieScanarePlanetara = false;

            if (Math.abs(tx) <= txDeadzone) {
                tureta.setPower(0);
                return;
            }

            // PID (daca merge invers, schimbi DOAR semnul aici)
            double error = tx;

            integral += error * dt;
            integral = Range.clip(integral, -1.0, 1.0);

            double derivative = (error - lastErrorT) / dt;
            lastErrorT = error;

            double output = kP * error + kI * integral + kD * derivative;
            output = Range.clip(output, -0.5, 0.5);

            output = Limitare(output);
            tureta.setPower(output);
            return;
        }

        // =========================
        // 2) TARGET INVALID
        // =========================
        double UltimaDataVazutSecunde = (OraActuala - UltimaDataVazut) / 1e9;

        if (aVazutVreodataTarget && UltimaDataVazutSecunde <= TimpPauza) {
            tureta.setPower(0);
            return;
        }

        // =========================
        // INIT SCAN (o singura data)
        // =========================
        if (TimpDeLaPierdereaTargetului == 0) {
            TimpDeLaPierdereaTargetului = OraActuala;

            integral = 0;
            lastErrorT = 0;

            // dupa pierdere: incearca intai spre last known
            if (aVazutVreodataTarget) scanDir = lastTx > 0;
            else scanDir = false; // prima data: stanga
        }

        ConditieScanarePlanetara = true;

        // =========================
        // 3) SWEEP WIDE (DAR fara sa intre in zona interzisa)
        // =========================
        double PutereCautare =Limitare(scanSpeed);
        tureta.setPower(PutereCautare);
    }

    private double Limitare(double power) {
        double angleDeg = getSpinnerPositionCorrected() * DEG_PER_TICK;

        // Limite hard
        double LEFT_LIMIT = -125;
        double RIGHT_LIMIT = 125;

        // Daca ajungi la limita -> inversezi directia
        if (angleDeg >= RIGHT_LIMIT) {
            scanDir = false;   // mergi inapoi spre stanga
        }
        else if (angleDeg <= LEFT_LIMIT) {
            scanDir = true;    // mergi inapoi spre dreapta
        }

        // Aplica directia corecta
        double finalPower = Math.abs(power) * (scanDir ? 1 : -1);

        return finalPower;
    }




    private String arrayToString(int[] a) {
        return "[" + a[0] + ", " + a[1] + ", " + a[2] + "]";
    }

    private void Lansare()
    {
        LLResult result = limelight.getLatestResult();
        double ty=result.getTy();

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 15.0;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightCM = 31.5;

        // distance from the target to the floor
        double goalHeightCM = 85;

        double angleToGoalDegrees = limelightMountAngleDegrees + ty;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        DistantaPerete=(goalHeightCM - limelightLensHeightCM) / Math.tan(angleToGoalRadians);
        double InaltimePerete=98.5;
        double InaltimeTarget=119;
        double DistantaTarget=DistantaPerete+45;
        double InaltimeRobot=35;
        double g=9.81; //constanta gravitationala


        double k=(DistantaTarget*(InaltimePerete-InaltimeRobot)-DistantaPerete*(InaltimeTarget-InaltimeRobot))/(DistantaTarget*DistantaPerete*(DistantaTarget-DistantaPerete));
        double u=(DistantaTarget*DistantaTarget*(DistantaPerete-DistantaTarget)-DistantaPerete*DistantaPerete*(InaltimeTarget-InaltimeRobot))/(DistantaTarget*DistantaPerete*(DistantaTarget-DistantaPerete));

        UnghiLansat=Math.toDegrees(Math.atan(u));

        //54grade limita inferioare de lansare din flywheel
        //75 grade limita superioara de lansare din flywheel

      //  UnghiLansat=Range.clip(UnghiLansat, 54.0, 75.0);

        //double PozitieLansare=(UnghiLansat-54.0)/(75.0-54.0);

       // UnghiLansare.setPosition(PozitieLansare);
    }



    private void Sort() {
      //  if (!gamepad1.yWasPressed()) return;
        if (spinnerMoving) return; // don't interrupt motion

        // Already sorted
        if (Arrays.equals(slots2, totem)) return;

        double stepTicks = (double) (120 * TICKS_PER_DEGREE);

        // Try rotating right
        rotateSlotsRight();
        if (Arrays.equals(slots2, totem)) {
            targetTicks += stepTicks;
            spinnerMoving = true;
            return;
        }
        // Undo rotation if not correct
        rotateSlotsLeft();

        // Try rotating left
        rotateSlotsLeft();
        if (Arrays.equals(slots2, totem)) {
            targetTicks -= stepTicks;
            spinnerMoving = true;
            return;
        }
        // Undo rotation if not correct
        rotateSlotsRight();

        // ❌ No match found
        telemetry.addLine("SORT FAILED: No matching orientation");
    }

    private int getSpinnerPositionCorrected() {
        return spinner.getCurrentPosition() + encoderOffset;
    }

    private void updateTelemetry() {
        telemetry.addData("Slot 1", slots2[0]);
        telemetry.addData("Slot 2", slots2[1]);
        telemetry.addData("Slot 3", slots2[2]);;
        telemetry.addData("ColorPending", colorPending);
        telemetry.addData("unghi tureta",tureta.getCurrentPosition() * DEG_PER_TICK);
        telemetry.addData("putere tureta",tureta.getPower());
        telemetry.addData("tx",lastTx);
        telemetry.addData("spinner unghi",getSpinnerPositionCorrected()*DEG_PER_TICK);
      //  telemetry.addData("DistantaTarget", DistantaPerete);
      //  telemetry.addData("unghiLansat",UnghiLansat);
    //    telemetry.addData("atingeriSenSor",atingeriSenzor);
     //   limit = hardwareMap.get(DigitalChannel.class, "limit");
       // limit.setMode(DigitalChannel.Mode.INPUT);
     //        telemetry.addData("Distanta Perete",DistantaPerete);
        telemetry.update();
    }





    @Override
    public void runOpMode() {
        InitWheels();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(orientation));


        colorsensorSLot1 = hardwareMap.colorSensor.get("Color1");
        // colorsensorSLot2 = hardwareMap.colorSensor.get("Color2");
        // colorsensorSLot3 = hardwareMap.colorSensor.get("Color3");

        spinner = hardwareMap.get(DcMotorEx.class, "spinner");
        intake = hardwareMap.get(DcMotor.class, "intake");
        ejector = hardwareMap.get(Servo.class, "ejector");
        tureta = hardwareMap.get(DcMotorEx.class, "tureta");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        limitswitch = hardwareMap.get(TouchSensor.class, "limitswitch");

        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        tureta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tureta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tureta.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();

        lastTime = System.nanoTime();
        pidTimer.reset();
        ejector.setPosition(0.285);

        while (opModeIsActive()) {

            // ---------------------------
            // CONTROL INTAKE
            // ---------------------------
            if (gamepad1.circleWasPressed()) {
                intake.setPower(-1);
                slots2[0] = 0;
                slots2[1] = 0;
                slots2[2] = 0;
                intakeM = true;
            }

            if (gamepad1.dpadDownWasPressed())
            {
                intake.setPower(1);
            }

            if (gamepad1.psWasPressed()) {
                intake.setPower(0);
             //   i = 0; // resetează indexul pentru slots2
            }

            if (gamepad1.optionsWasReleased()) {
                ejector.setPosition(0.285);
            } else if (gamepad1.optionsWasPressed()) {
                ejector.setPosition(0.005);

            }
            if (gamepad1.shareWasPressed()) {
                flywheelOn = !flywheelOn;
            }

            double flywheelPower = flywheelOn ? logFlywheel(flywheelInput) : 0;
            if (flywheelOn) flywheel.setPower(0.55);
            else flywheel.setPower(0);

            if (gamepad1.squareWasPressed()) {
                targetTicks +=  (60 * TICKS_PER_DEGREE);
            }

            if (gamepad1.rightBumperWasPressed()) {
                targetTicks +=  (30 * TICKS_PER_DEGREE);
            }


            // ---------------------------
            // CONTROL MANUAL SPINNER
            // ---------------------------
            if (gamepad1.dpadRightWasPressed()) {
                targetTicks +=  (120 * TICKS_PER_DEGREE);
                rotateSlotsRight();
            }

            if (gamepad1.dpadLeftWasPressed()) {
                targetTicks -=  (120 * TICKS_PER_DEGREE);
                rotateSlotsLeft();
            }
            if (gamepad1.leftBumperWasPressed()) {
                outtakeActive = true;
            }




            double currentPos = getSpinnerPositionCorrected();
            double error = targetTicks - currentPos;
            integralSum += error;
            double derivative = error - lastError;
            lastError = error;
            double pidOutput = error * P + integralSum * I + derivative * D;
            pidOutput = Math.max(-0.5, Math.min(0.5, pidOutput));
            if(!reset)spinner.setPower(pidOutput);

            if (Math.abs(error) < 50 && Math.abs(pidOutput) < 0.05) {
                spinnerMoving = false;
                detectionLocked = false;   // 🔓 UNLOCK HERE
                lastStableColorSensor1 = 0;
            }

            // ---------------------------
// START OUTTAKE WHEN FULL (ONE-SHOT)
// ---------------------------
            if (spinnerIsFull() && !wasFull && !outtakeActive) {
                outtakeActive = true;

                ballsOuttaken = 0;
                ejectOpen = false;
                ejectClose = false;
                rotating = false;

                sorted = false;
                finalMoveDone = false;

                outtakeStepTimer.reset();
            }


            if (!spinnerIsFull() && !spinnerMoving && !outtakeActive) {
                updateAllSlots();

                if (waitingForClear && slots[0] == 0) {
                    waitingForClear = false;
                    lastStableColorSensor1 = 0;
                }

                colorDrivenSpinnerLogic();
            }


            if (spinnerIsFull()) {

                if (!sorted && outtakeStepTimer.milliseconds() >= 3500) {
                    Sort();
                    sorted = true;
                }

                if (sorted
                        && outtakeStepTimer.milliseconds() >= 7000
                        && !finalMoveDone
                ) {
                    targetTicks +=  (60 * TICKS_PER_DEGREE);
                    spinnerMoving = true;
                    finalMoveDone = true;
                }

            } else {
                sorted = false;
            }

            /*if (limit.isPressed())atingeriSenzor= (atingeriSenzor+1)%5;
            if (atingeriSenzor==0)
            {
                spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                I=0;
            }*/
            if (limitswitch.isPressed()) {
                if (!reset) {
                    CalibrareSpinner();
                    reset = true;
                }
            } else {
                reset = false;
            }
                //   if (gamepad1.touchpadWasPressed()) reset = true;
/*
            if (gamepad1.touchpadWasPressed()) reset = true;

            if (reset) {
                spinner.setPower(0.15);  // move slowly toward limit switch
                if (limitswitch.isPressed()) {
                    spinner.setPower(0);  // stop motor
                    spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    targetTicks = 0;      // reset PID target
                    integralSum = 0;      // reset PID integral
                    lastError = 0;        // reset PID derivative
                    reset = false;        // done resetting
                    targetTicks -=  (5 * TICKS_PER_DEGREE);
                    telemetry.addData("senzor",limitswitch.isPressed());
                }
            }


*/


                updateTelemetry();
            SetWheelsPower();
            if (gamepad1.yWasPressed()) {
                activtureta = !activtureta;
            }

            if (activtureta) {
                runAiming();
                Lansare();
            } else {
                tureta.setPower(0);
            }
            simpleOuttake();
            wasFull = spinnerIsFull();

            idle();
            if (!spinnerMoving) integralSum = 0;
        }
//Tibichi:ce ai avut cu functia lansare in contextu in care nu facea nimic fizic pe robot :(
    }
}
