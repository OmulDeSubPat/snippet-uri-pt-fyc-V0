package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "&&TeleOpMainBlueClose")
public class CipTeleop extends LinearOpMode {

    /* ===================== FAST INTAKE DETECT ===================== */
    static final long DETECT_DELAY_MS = 0;       // immediate once stable
    static final long SERVO_MOVE_LOCK_MS = 45;   // shorter lock for faster cycling
    long colorStartTimeMs = 0;
    long servoMoveStartMs = 0;

    int[] last5Sensor1 = new int[5];
    int[] last5Sensor2 = new int[5];
    int[] last5Sensor3 = new int[5];
    int indexSensor1 = 0;
    int indexSensor2 = 0;
    int indexSensor3 = 0;

    int[] slots = new int[3];
    int[] totem = {2, 1, 2};

    // Color tracking
    int Color1 = 0;
    int Color2 = 0;
    int Color3 = 0;
    int detectedBalls = 0;
    int slotIntakeIndex = 0;
    double prev_t_outtake = 0;
    double prev_t_intake = 0;

    ColorSensor colorsensorSLot1;
    ColorSensor colorsensorSLot2;
    ColorSensor colorsensorSLot3;

    DcMotor intake;
    DcMotorEx tureta;
    Servo ejector;
    Servo trajectoryAngleModifier;

    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;

    DcMotorEx flywheel;

    Servo spinnerCLose;
    Servo spinnerFar;

    boolean spinIntake = false;

    Limelight3A limelight;
    IMU imu;

    boolean flywheelOn = false;
    boolean intakeMode = false;
    boolean intakeReverse = false;
    boolean outtakeMode = false;

    private ElapsedTime outtakeTimeout = new ElapsedTime();
    private ElapsedTime intakeTimeout = new ElapsedTime();

    final double ejectorDown = 0.19;
    final double ejectorUp = 0.02;

    double t_intake = 0;

    // NOTE: you use modulo 4 in logic, so keep 4 entries
    final double[] slotPositionsIntake = {0, 0.19, 0.38, 0.085};

    PinpointLocalizer pinpoint;
    Pose pose;

    double CoordX, CoordY, header;

    double Posspinner = 0;
    double PosspinnerMin = 0;
    double PosspinnerMax = 0.95;

    int ballsLoaded = 0;
    final int minFlywheelRPM = 1000;

    static final double FLYWHEEL_TICKS_PER_REV = 28;

    double flywheelPowerHigh = 0.65;
    double flywheelPowerLow = 0.55;
    double flywheelTolerance = 20; // RPM

    /* ===================== TURRET GEOMETRY / LIMITS ===================== */
    private static final double MOTOR_TICKS_PER_REV = 384.5;
    private static final double MOTOR_TO_TURRET_RATIO = 76.0 / 24.0;

    private static final double DEG_PER_TICK_TURETA =
            360.0 / (MOTOR_TICKS_PER_REV * MOTOR_TO_TURRET_RATIO);

    // Turret soft limits (degrees)
    private static final double LEFT_LIMIT  = -110;
    private static final double RIGHT_LIMIT = 110;

    // Control
    private static final double kP = 0.015;
    private static final double MAX_POWER_TURETA = 0.2;
    private static final double TURRET_DEADBAND_DEG = 1.0; // prevents hunting drift

    double targetX = 12;
    double targetY = 136;

    double turretX = 0.0;
    double turretY = 0.0;

    double power = 0;

    // IMPORTANT: if encoder=0 means turret angle=0°, then startTurretAngle MUST be 0
    final double startTurretAngle = 0.0;

    double trajectoryAngle = 73.2;
    int flywheelTargetRPM = 0;

    /* ================= LOCALIZATION ================= */
    private Pose startPose;
    private double pX, pY;

    /* ================= TARGET / LAUNCH ================= */
    private boolean aimingEnabled = false;
    private boolean launcherEnabled = false;

    final double[][] launchZoneBig = {{-18,144},{162,144},{72,55}};
    final double[][] launchZoneSmall = {{40,0},{102,0},{72,32}};

    final double absoluteTurretHeight = 0.35; // meters
    final double absoluteTargetHeight = 1.1;  // meters
    final double relativeHeight = absoluteTargetHeight - absoluteTurretHeight;
    final double prefferedMaxHeightThrow = relativeHeight + 0.3; // meters
    final double launcherEfficiency = 0.425; // needs experimenting
    final double flywheelRadius = 0.048;
    final double g = 9.81;

    final double trajectoryAngleModifierGearRatio = 123/15.0;
    final double trajectoryAnglerMaxTravel = 300.0;
    final double trajectoryAnglePosPerDegree = trajectoryAngleModifierGearRatio / trajectoryAnglerMaxTravel;

    final double minTrajectoryAngle = 53.2;
    final double maxTrajectoryAngle = 73.2;

    final double turretOffsetX = 0.0;
    final double turretOffsetY = 52/1000.0;

    final double TICKS_PER_REV_FLYWHEEL = 28;
    final int maxFlywheelRPM = 6000;

    final int telemetryDelay = 200;
    private ElapsedTime telemetryTimer = new ElapsedTime();

    int outtakeStep = 0;

    int[] logicalSlots = new int[3];
    int[] lastNIntake = new int[5];
    int idxIntake = 0;
    boolean spinnerMoving = false;
    boolean detectionLocked = false;
    boolean colorPending = false;
    boolean waitingForClear = false;
    int lastStableIntakeColor = 0;

    double error = 0;
    double targetTurretDeg = 0;
    double currentTurretDeg = 0;

    boolean enabledSorting = false;
    int motifIndex = 0;
    int nextOuttakeSlot = -1;

    int[] OuttakeSlotColors = {0,0,0};
    int[] Motif = {1,2,1};
    int[] IntakeSlotColors = {0,0,0};

    /* ===================== LAUNCHER / ANGLE ===================== */
    private void updateLauncher(){
        double ticksPerSecond = flywheelTargetRPM * TICKS_PER_REV_FLYWHEEL / 60.0;
        flywheel.setVelocity(ticksPerSecond);
    }

    private void updateTrajectoryAngle(){
        setTrajectoryAngle(trajectoryAngle);
    }

    private void computeParameters() {
        double d = Math.hypot(targetX - turretX, targetY - turretY) * 0.0254; // inches->m

        if (d <= 1e-6) {
            flywheelTargetRPM = 3000;
            trajectoryAngle = maxTrajectoryAngle;
            return;
        }

        double k = (4.0 * prefferedMaxHeightThrow / d)
                * (1.0 - Math.sqrt(1.0 - relativeHeight / prefferedMaxHeightThrow));

        double idealAngle = Math.toDegrees(Math.atan(k));

        boolean constrained = (idealAngle > maxTrajectoryAngle || idealAngle < minTrajectoryAngle);

        if (constrained) {
            trajectoryAngle = (idealAngle > maxTrajectoryAngle) ? maxTrajectoryAngle : minTrajectoryAngle;

            double thetaRad = Math.toRadians(trajectoryAngle);
            double cosTheta = Math.cos(thetaRad);
            double tanTheta = Math.tan(thetaRad);

            if (Math.abs(cosTheta) < 1e-6) return;

            double denominator = d * tanTheta - relativeHeight;
            if (denominator <= 0) {
                flywheelTargetRPM = maxFlywheelRPM;
                return;
            }

            double exitVelocity = Math.sqrt((g * d * d) /
                    (2 * cosTheta * cosTheta * denominator));

            flywheelTargetRPM = (int)(60 * exitVelocity /
                    (2 * Math.PI * flywheelRadius * launcherEfficiency));

        } else {
            trajectoryAngle = idealAngle;

            double thetaRad = Math.toRadians(trajectoryAngle);
            double sinTheta = Math.sin(thetaRad);
            if (Math.abs(sinTheta) < 1e-6) return;

            double v0 = Math.sqrt(2 * g * prefferedMaxHeightThrow) / sinTheta;
            flywheelTargetRPM = (int)(60 * v0 / (2 * Math.PI * flywheelRadius * launcherEfficiency));
        }

        flywheelTargetRPM = Math.max(minFlywheelRPM, Math.min(flywheelTargetRPM, maxFlywheelRPM));
    }

    private void setTrajectoryAngle(double angle){
        angle = Range.clip(angle, minTrajectoryAngle, maxTrajectoryAngle);
        double position = (angle - minTrajectoryAngle) * trajectoryAnglePosPerDegree;
        trajectoryAngleModifier.setPosition(position);
    }

    /* ===================== LAUNCH ZONE ENABLE ===================== */
    public void disableIfNotInLaunchZone(){
        if (pose == null) return;

        double robotX = pose.getX();
        double robotY = pose.getY();

        double d1, d2, d3;
        if(robotY > 48) {
            d1 = sign(robotX, robotY, launchZoneBig[0][0], launchZoneBig[0][1], launchZoneBig[1][0], launchZoneBig[1][1]);
            d2 = sign(robotX, robotY, launchZoneBig[1][0], launchZoneBig[1][1], launchZoneBig[2][0], launchZoneBig[2][1]);
            d3 = sign(robotX, robotY, launchZoneBig[2][0], launchZoneBig[2][1], launchZoneBig[0][0], launchZoneBig[0][1]);
        } else {
            d1 = sign(robotX, robotY, launchZoneSmall[0][0], launchZoneSmall[0][1], launchZoneSmall[1][0], launchZoneSmall[1][1]);
            d2 = sign(robotX, robotY, launchZoneSmall[1][0], launchZoneSmall[1][1], launchZoneSmall[2][0], launchZoneSmall[2][1]);
            d3 = sign(robotX, robotY, launchZoneSmall[2][0], launchZoneSmall[2][1], launchZoneSmall[0][0], launchZoneSmall[0][1]);
        }

        boolean has_neg = d1 < 0 || d2 < 0 || d3 < 0;
        boolean has_pos = d1 > 0 || d2 > 0 || d3 > 0;

        if (has_neg && has_pos){
            disableAiming();
        } else {
            if(!aimingEnabled) enableAiming();
        }
    }

    private double sign(double x1,double y1,double x2,double y2,double x3,double y3){
        return (x1-x3)*(y2-y3)-(x2-x3)*(y1-y3);
    }

    public void enableLauncher(){
        if(launcherEnabled) return;
        launcherEnabled = true;
        flywheelTargetRPM = 3000;
        updateLauncher();
    }

    public void disableLauncher(){
        if(!launcherEnabled) return;
        launcherEnabled = false;
        flywheelTargetRPM = 0;
        updateLauncher();
    }

    public void enableAiming(){
        aimingEnabled = true;
    }

    public void disableAiming(){
        aimingEnabled = false;
    }

    /* ===================== LOCALIZATION INIT ===================== */
    private void initLocalization() {
        pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        startPose = new Pose(22, 127, Math.toRadians(-36));
        pinpoint.setStartPose(startPose);
    }

    private void resetLocalization(){
        pinpoint.setPose(startPose);
    }

    /* ===================== SLOTS MAPPING (unused here but kept) ===================== */
    private void mapIntakeToOuttakeSlots() {
        for (int i = 0; i < 3; i++) {
            OuttakeSlotColors[2 - i] = IntakeSlotColors[i];
        }
    }

    private void selectNextOuttakeSlot() {
        nextOuttakeSlot = -1;

        if(enabledSorting) {
            for (int i = 0; i < 3; i++) {
                if (OuttakeSlotColors[i] == Motif[motifIndex]) {
                    nextOuttakeSlot = i;
                    break;
                }
            }
            motifIndex = (motifIndex + 1) % Motif.length;
        }

        if (nextOuttakeSlot == -1) {
            for (int i = 0; i < 3; i++) {
                if (OuttakeSlotColors[i] != 0) {
                    nextOuttakeSlot = i;
                    break;
                }
            }
        }
    }

    /* ===================== HARDWARE INIT ===================== */
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

    private void InitDc() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        tureta = hardwareMap.get(DcMotorEx.class, "tureta");
        tureta.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        tureta.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        tureta.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(20, 5, 12, 18);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void InitServo() {
        ejector = hardwareMap.get(Servo.class, "ejector");
        spinnerFar = hardwareMap.get(Servo.class, "SpinnerFar");
        spinnerCLose = hardwareMap.get(Servo.class, "SpinnerClose");
        trajectoryAngleModifier = hardwareMap.get(Servo.class, "unghituretaoy");

        ejector.setDirection(Servo.Direction.REVERSE);

        ejector.setPosition(ejectorDown);
        spinnerFar.setPosition(0);
        spinnerCLose.setPosition(0);
    }

    private void InitLL() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(orientation));
    }

    private void InitAux() {
        colorsensorSLot1 = hardwareMap.colorSensor.get("Color1");
        colorsensorSLot2 = hardwareMap.colorSensor.get("Color2");
        colorsensorSLot3 = hardwareMap.colorSensor.get("Color3");
        // DO NOT re-init pinpoint here; initLocalization() will do it.
    }

    /* ===================== DRIVE ===================== */
    private void SetWheelsPower() {
        double left_x = gamepad1.left_stick_x;
        double left_y = -gamepad1.left_stick_y; // forward is negative
        double right_x = gamepad1.right_stick_x;

        double front_left_pw = left_y + left_x + right_x;
        double back_left_pw = left_y - left_x + right_x;
        double front_right_pw = left_y - left_x - right_x;
        double back_right_pw = left_y + left_x - right_x;

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

    /* ===================== ANGLES ===================== */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /* ===================== TURRET AIM (FIXED) ===================== */
    private void updateTurretAim() {
        if (pose == null) return;

        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = pose.getHeading();
        double robotHeadingDeg = Math.toDegrees(robotHeading);

        // turret position in field (if you want offsets)
        turretX = robotX + turretOffsetX*Math.cos(robotHeading) - turretOffsetY*Math.sin(robotHeading);
        turretY = robotY + turretOffsetX*Math.sin(robotHeading) + turretOffsetY*Math.cos(robotHeading);

        double dx = targetX - turretX;
        double dy = targetY - turretY;

        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));

        // ENCODER 0 => 0 degrees (your statement), so offset must be 0
        currentTurretDeg = (tureta.getCurrentPosition() * DEG_PER_TICK_TURETA) + startTurretAngle;
        currentTurretDeg = normalizeAngle(currentTurretDeg);

        if (aimingEnabled) {
            targetTurretDeg = normalizeAngle(fieldAngle - robotHeadingDeg);
        } else {
            // park at 0°, not -180°
            targetTurretDeg = 0.0;
        }

        // FIX: correct soft-limit clamp
        targetTurretDeg = Range.clip(targetTurretDeg, LEFT_LIMIT, RIGHT_LIMIT);

        error = normalizeAngle(targetTurretDeg - currentTurretDeg);

        // deadband to prevent “drift” from localization noise / quantization
        if (Math.abs(error) < TURRET_DEADBAND_DEG) {
            tureta.setPower(0);
            power = 0;
            return;
        }

        power = Range.clip(error * kP, -MAX_POWER_TURETA, MAX_POWER_TURETA);
        tureta.setPower(power);
    }

    /* ===================== COLOR ===================== */
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

    private int smekerie1(ColorSensor colorSensor) {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        int alpha = colorSensor.alpha();
        double h = getHue(r, g, b);

        int detected;
        if (alpha < 100 && (h == 150 || h == 144)) detected = 0;
        else if ((h > 215) || (alpha < 100 && (h == 160 || h == 180))) detected = 2;
        else if (h > 135 && h < 160 && alpha > 100) detected = 1;
        else if ((h == 140 || h == 145) && alpha == 43) detected = 0;
        else if (h > 135 && h < 160 && alpha > 60) detected = 1;
        else if ((h == 210 || h == 220 || h == 225 || h == 200) && alpha < 100) detected = 2;
        else detected = 0;

        return detected;
    }

    private int CuloareFinala1(ColorSensor sensor, int[] last5, int index) {
        last5[index] = smekerie1(sensor);

        int count1 = 0, count2 = 0;
        for (int v : last5) {
            if (v == 1) count1++;
            else if (v == 2) count2++;
        }

        if (count1 >= 3) return 1;
        if (count2 >= 3) return 2;
        return 0;
    }

    private void updateCulori() {
        Color1 = CuloareFinala1(colorsensorSLot1, last5Sensor1, indexSensor1);
        indexSensor1 = (indexSensor1 + 1) % 5;

        Color2 = CuloareFinala1(colorsensorSLot2, last5Sensor2, indexSensor2);
        indexSensor2 = (indexSensor2 + 1) % 5;

        Color3 = CuloareFinala1(colorsensorSLot3, last5Sensor3, indexSensor3);
        indexSensor3 = (indexSensor3 + 1) % 5;
    }

    /* ===================== MANUAL SPINNER ===================== */
    private void servoLogic() {
        if (gamepad1.touchpadWasPressed())
            Posspinner = 0;

        if (gamepad1.dpadRightWasPressed()) {
            slotIntakeIndex++;
            slotIntakeIndex = slotIntakeIndex % 3;
            Posspinner = slotPositionsIntake[slotIntakeIndex];
        }

        if (gamepad1.dpadLeftWasPressed()){
            slotIntakeIndex--;
            if(slotIntakeIndex < 0) slotIntakeIndex = 2;
            Posspinner = slotPositionsIntake[slotIntakeIndex];
        }
    }

    private boolean spinnerFull() {
        detectedBalls = 0;
        if (Color1 != 0) detectedBalls++;
        if (Color2 != 0) detectedBalls++;
        if (Color3 != 0) detectedBalls++;
        return detectedBalls == 3;
    }

    private double getFlywheelRPM() {
        return flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    /* ===================== LOGICAL SLOT ROTATE ===================== */
    private void rotateLogicalSlotsRight() {
        int temp = logicalSlots[2];
        logicalSlots[2] = logicalSlots[1];
        logicalSlots[1] = logicalSlots[0];
        logicalSlots[0] = temp;
    }

    // ULTRA-FAST intake smoothing
    private int processIntakeSensor(ColorSensor sensor) {
        int detected = smekerie1(sensor);

        lastNIntake[idxIntake] = detected;
        idxIntake = (idxIntake + 1) % lastNIntake.length;

        int count1 = 0, count2 = 0;
        for (int v : lastNIntake) {
            if (v == 1) count1++;
            else if (v == 2) count2++;
        }

        int finalColor = 0;
        if (count1 >= 2 && count1 > count2) finalColor = 1;
        else if (count2 >= 2 && count2 > count1) finalColor = 2;

        return finalColor;
    }

    private void colorDrivenSpinnerLogicServos() {

        if (spinnerMoving) {
            if (System.currentTimeMillis() - servoMoveStartMs >= SERVO_MOVE_LOCK_MS) {
                spinnerMoving = false;
                detectionLocked = false;
                lastStableIntakeColor = 0;
            } else {
                return;
            }
        }

        if (waitingForClear) {
            int intakeColorNow = processIntakeSensor(colorsensorSLot1);
            if (intakeColorNow == 0) {
                waitingForClear = false;
                lastStableIntakeColor = 0;
            }
            return;
        }

        int intakeColor = processIntakeSensor(colorsensorSLot1);

        boolean newColorDetected = (intakeColor != 0 && lastStableIntakeColor == 0);
        if (newColorDetected) {
            lastStableIntakeColor = intakeColor;
        }

        if (newColorDetected && !colorPending && !detectionLocked) {
            colorStartTimeMs = System.currentTimeMillis();
            colorPending = true;
        }

        if (colorPending && (System.currentTimeMillis() - colorStartTimeMs >= DETECT_DELAY_MS)) {

            logicalSlots[0] = intakeColor;
            rotateLogicalSlotsRight();

            slotIntakeIndex++;
            slotIntakeIndex = slotIntakeIndex % 4;
            Posspinner = slotPositionsIntake[slotIntakeIndex];

            waitingForClear = true;
            detectionLocked = true;
            spinnerMoving = true;
            servoMoveStartMs = System.currentTimeMillis();

            colorPending = false;
        }
    }

    /* ===================== TELEMETRY ===================== */
    private void updateTelemetry() {
        if(telemetryTimer.milliseconds() >= telemetryDelay) {

            if (outtakeMode) {
                telemetry.addData("Slot 1", slots[0]);
                telemetry.addData("Slot 2", slots[1]);
                telemetry.addData("Slot 3", slots[2]);
            }
            if (intakeMode) {
                telemetry.addData("Slot 1", Color1);
                telemetry.addData("Slot 2", Color2);
                telemetry.addData("Slot 3", Color3);
            }

            telemetry.addData("timp_outtake", outtakeTimeout.time());

            if (pose != null) {
                telemetry.addData("x", pose.getX());
                telemetry.addData("y", pose.getY());
                telemetry.addData("heading", Math.toDegrees(pose.getHeading()));
            }

            telemetry.addData("SpinnerPos", spinnerFar.getPosition());
            telemetry.addData("balls", detectedBalls);

            telemetry.addData("Sensor 1a", colorsensorSLot1.alpha());
            telemetry.addData("Sensor 2a", colorsensorSLot2.alpha());
            telemetry.addData("Sensor 3a", colorsensorSLot3.alpha());

            telemetry.addData("Hue1", getHue(colorsensorSLot1.red(), colorsensorSLot1.green(), colorsensorSLot1.blue()));
            telemetry.addData("Hue2", getHue(colorsensorSLot2.red(), colorsensorSLot2.green(), colorsensorSLot2.blue()));
            telemetry.addData("Hue3", getHue(colorsensorSLot3.red(), colorsensorSLot3.green(), colorsensorSLot3.blue()));

            telemetry.addData("TargetTurret", "%.1f", targetTurretDeg);
            telemetry.addData("TurretDeg", "%.1f", currentTurretDeg);
            telemetry.addData("Error", "%.1f", error);
            telemetry.addData("Power", "%.2f", power);

            telemetry.addData("Flywheel Target RPM", flywheelTargetRPM);
            telemetry.addData("Flywheel RPM", getFlywheelRPM());

            telemetry.addData("trajectoryAngle", trajectoryAngle);

            telemetry.update();
            telemetryTimer.reset();
        }
    }

    /* ===================== INTAKE/OUTTAKE ===================== */
    private void runIntake(){
        t_intake = intakeTimeout.milliseconds();
        if(t_intake >= prev_t_intake) {
            updateCulori();
            colorDrivenSpinnerLogicServos();
        }
    }

    private void runOuttake() {
        intake.setPower(1);

        final int EJECTOR_UP_DELAY = 250;
        final int EJECTOR_DOWN_DELAY = 200;
        final int SPINNER_SLOT_CHANGE_DELAY = 300;
        final int INITIAL_DELAY = 200;

        slots[0] = Color1;
        slots[1] = Color2;
        slots[2] = Color3;

        Color1 = 0;
        Color2 = 0;
        Color3 = 0;

        double t = outtakeTimeout.milliseconds();
        if(t >= prev_t_outtake){
            switch (outtakeStep++) {
                case 0:
                    Posspinner = 0.09;
                    prev_t_outtake = t + INITIAL_DELAY;
                    break;
                case 1:
                case 7:
                case 4:
                    ejector.setPosition(ejectorUp);
                    prev_t_outtake = t + EJECTOR_UP_DELAY;
                    break;
                case 2:
                case 8:
                case 5:
                    ejector.setPosition(ejectorDown);
                    prev_t_outtake = t + EJECTOR_DOWN_DELAY;
                    break;
                case 3:
                    Posspinner = 0.29;
                    prev_t_outtake = t + SPINNER_SLOT_CHANGE_DELAY;
                    break;
                case 6:
                    Posspinner = 0.47;
                    prev_t_outtake = t + SPINNER_SLOT_CHANGE_DELAY;
                    break;
                case 9:
                    Posspinner = 0;
                    outtakeStep = 0;
                    outtakeMode = false;
                    intakeMode = true;
                    ballsLoaded = 0;
                    prev_t_outtake = 0;
                    break;
            }
        }
    }

    /* ===================== MAIN ===================== */
    @Override
    public void runOpMode () {
        InitWheels();
        InitAux();
        InitDc();
        InitLL();
        InitServo();
        initLocalization();

        waitForStart();

        while (opModeIsActive()) {

            // apply spinner positions
            if (Posspinner >= PosspinnerMin && Posspinner <= PosspinnerMax) {
                spinnerFar.setPosition(Posspinner);
                spinnerCLose.setPosition(Posspinner);
            }

            servoLogic();
            SetWheelsPower();

            pinpoint.update();
            pose = pinpoint.getPose();

            disableIfNotInLaunchZone();
            updateTurretAim();
            updateTelemetry();

            if(aimingEnabled){
                computeParameters();
                updateLauncher();
                updateTrajectoryAngle();
            }

            // launcher auto enable if full & not aiming
            if(spinnerFull() && !aimingEnabled){
                enableLauncher();
            } else {
                if(!spinnerFull() && launcherEnabled && !aimingEnabled) disableLauncher();
            }

            // intake reverse on cross press
            if (gamepad1.crossWasPressed() && !gamepad1.crossWasReleased()){
                intake.setPower(-1);
            } else if (gamepad1.crossWasReleased()) {
                intake.setPower(spinIntake ? 1:0);
            }

            // reset localization
            if(gamepad1.optionsWasPressed() && gamepad1.shareWasPressed()){
                resetLocalization();
            }

            // toggle intake mode
            if (gamepad1.circleWasPressed()) {
                intakeTimeout.reset();
                prev_t_intake = 0;

                intakeMode = true;
                spinIntake = !spinIntake;

                intake.setPower(spinIntake ? 1 : 0);
                outtakeMode = false;

                ballsLoaded = 0;
                Posspinner = 0;
                slotIntakeIndex = 0;

                logicalSlots[0] = 0;
                logicalSlots[1] = 0;
                logicalSlots[2] = 0;

                waitingForClear = false;
                detectionLocked = false;
                spinnerMoving = false;
                colorPending = false;
                lastStableIntakeColor = 0;

                for (int i = 0; i < lastNIntake.length; i++) lastNIntake[i] = 0;
                idxIntake = 0;
            }

            if (gamepad1.yWasPressed()) {
                intake.setPower(0);
            }

            if (intakeMode && !outtakeMode) {
                runIntake();
            }

            // outtake
            if (gamepad1.right_trigger > 0.8) {
                if(!outtakeMode) {
                    outtakeTimeout.reset();
                    outtakeStep = 0;
                }
                outtakeMode = true;
                intakeMode = false;
                intake.setPower(0);
            }

            if (outtakeMode) {
                runOuttake();
            }
        }
    }
}
