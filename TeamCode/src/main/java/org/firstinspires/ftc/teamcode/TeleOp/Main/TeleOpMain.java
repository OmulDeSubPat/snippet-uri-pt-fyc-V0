package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOpServosAndPivoter")
public class TeleOpMain extends LinearOpMode {

    private DriveTrain driveTrain;
    private TurretAiming turretAiming;
    private IntakeOuttakeSystem intakeSystem;
    private ColorSensorProcessor colorProcessor;

    @Override
    public void runOpMode() {
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        turretAiming = new TurretAiming(hardwareMap, this);
        intakeSystem = new IntakeOuttakeSystem(hardwareMap, gamepad2);
        colorProcessor = new ColorSensorProcessor(hardwareMap);

        waitForStart();

        Thread wheelThread = new Thread(driveTrain::runDrive);
        Thread systemsThread = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                intakeSystem.update();
                //colorProcessor.updateTelemetry(telemetry);
            }
        });
        Thread aimThread = new Thread(turretAiming::runAiming);

        wheelThread.start();
        systemsThread.start();
        aimThread.start();

        while (opModeIsActive() && !isStopRequested()) {
            idle();
        }
    }
}