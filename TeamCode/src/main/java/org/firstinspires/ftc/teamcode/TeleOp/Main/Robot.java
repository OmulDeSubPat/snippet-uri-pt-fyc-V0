package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Robot {

    public Drive drive;
    public Poses poses;
    public Turret turret;
    public Vision vision;
    public ColorSystem colorSystem;

    public Robot(LinearOpMode opMode) {
        drive = new Drive(opMode);
        poses = new Poses(opMode);
        turret = new Turret(opMode);
        vision = new Vision(opMode, turret);
        colorSystem = new ColorSystem(opMode);
    }

    public void init() {

    }

    public void servoInit() {
    }
}
