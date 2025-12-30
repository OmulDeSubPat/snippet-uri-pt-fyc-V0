package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="TeleOpMainPeClase")
public class TeleOpMain extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() {

        robot = new Robot(this);

        robot.drive.InitWheels();
        robot.poses.DcInit();
        robot.poses.servoInit();
        robot.turret.init();
        robot.vision.limeInit();

        waitForStart();

        robot.turret.lastTime = System.nanoTime();

        Thread chassisThread = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                robot.drive.SetWheelsPower();
                Thread.yield();
            }
        });

        Thread systemsThread = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                robot.poses.pozitii();
                robot.vision.aliniereTureta();
                Thread.yield();
            }
        });

        chassisThread.start();
        systemsThread.start();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Turret Angle",
                    robot.turret.MotorTureta.getCurrentPosition() * robot.turret.DEG_PER_TICK);
            telemetry.update();
            idle();
        }
    }
}
