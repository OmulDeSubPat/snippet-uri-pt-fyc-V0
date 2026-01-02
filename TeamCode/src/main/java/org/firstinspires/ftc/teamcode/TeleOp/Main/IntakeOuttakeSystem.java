package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeOuttakeSystem {
    Servo ejector ;//servo ul care baga mingile in flywheel
    Servo ungiTuretaOy;//cea care asigura ca unghiul turetei e pe apriltag pe oy
    //nici macar nu stiu daca mai exista
    DcMotor matura;//intake ul activ, (ala pasiv unde e? ~tibichi)
    DcMotor flywheel;//cel care lanseaza mingea
    DcMotor spinner;//cel care roteste mingile in rezervor

    private final Gamepad gamepad;

    public IntakeOuttakeSystem(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;

        ejector = hardwareMap.servo.get("ejector");
        ungiTuretaOy = hardwareMap.servo.get("unghituretaoy");

        matura = hardwareMap.dcMotor.get("matura");

        flywheel = hardwareMap.dcMotor.get("flywheel");
        flywheel.setPower(0.3);//pentru a nu avea un cold start TODO de tunat empiric

        spinner = hardwareMap.dcMotor.get("spinner");
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setTargetPosition(0);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(0);
    }

    public void update() {
        //intake
        if (gamepad.circleWasPressed())
        {
            matura.setPower(1);
            flywheel.setPower(0.3);
        }

        //outtake
        if (gamepad.xWasPressed())
        {
            flywheel.setPower(1);
            matura.setPower(0);
            ejector.setPosition(0.555);
            //TODO sa se miste 120 de grade spinner ul + gandit spinner autonom
        }
    }
}