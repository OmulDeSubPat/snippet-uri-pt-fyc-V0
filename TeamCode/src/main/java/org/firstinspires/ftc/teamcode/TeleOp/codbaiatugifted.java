//4 dc sasiu - clasic
//dc maturica power 1 0
//dc spinner (revolver) PID cu 3 limitswich uri sincronizat cu senzorii de culaore
//la tureta dc care contrealaza pivotu
//motor de turatie Dc
//ajustare unghi servo
//servo jos eject spinner range 150 grade
//servo sus ajustare unghi tureta
//
//pozitii:
//intake activ (dc maturica activ -> toate slot urile sunt ocupate +stiind ordinea)
//scoring(ajustare unghi servo-> se duce in pozitie , pivotu->urmareste cosul prin limelight ,
// motor turatie (pornit putin mai devreme , invartit constant la viteza mai mica 0.6,
// cand is 3 slot uri pline dau putere 1) -> activ , eject bila mutat spinner.


package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class codbaiatugifted extends LinearOpMode{
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor matura;
    DcMotor pivoter;
    DcMotor turatie;
    DcMotor spinner;
    Servo unghi;
    int limitswitch = 0;
    int position = 0;
    int encoderPosition = 0;
    int targetPositionSpinner = 0;
    double unghiTargetPosition = 0;
    double pivoterTargetPosition = 0;
    double error = 0;
    double lastError = 0;
    double derivative = 0;
    int Kp = 0;
    int Ki = 0;
    int Kd = 0;
    double integralSum = 0;
    double out = 0;
    TouchSensor touch;

    int slotIndex = 0;

    private void InitWheels() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftRear");
        rightBack= hardwareMap.dcMotor.get("rightRear");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
    }

    private void InitDc() {
        matura =hardwareMap.dcMotor.get("matura");
        pivoter =hardwareMap.dcMotor.get("pivoter");
        turatie =hardwareMap.dcMotor.get("turatie");
        spinner = hardwareMap.dcMotor.get("revolver");


    }

    private void InitServo() {
        unghi = hardwareMap.servo.get("unghi_tureta");
        touch = hardwareMap.get(TouchSensor.class, "limitSwitch");
    }

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

        leftFront.setPower(front_left_pw);
        leftBack.setPower(back_left_pw);
        rightFront.setPower(front_right_pw);
        rightBack.setPower(back_right_pw);
    }
    private void intake() {
        matura.setPower(1.0);
    }

    private void revolver() {
        ElapsedTime timer = new ElapsedTime();
        while(!touch.isPressed()){
            encoderPosition = spinner.getCurrentPosition();
            error = targetPositionSpinner - encoderPosition;


            derivative = (error - lastError) / timer.seconds();
            integralSum = integralSum + (error * timer.seconds());

            out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            spinner.setPower(out);

            lastError = error;

            timer.reset();
        }
    }

    private void score() {
        unghi.setPosition(unghiTargetPosition / 270);

        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while (timer.seconds()>3 && opModeIsActive())// nu stiu exact
        while(pivoterTargetPosition - pivoter.getCurrentPosition() != 0) {
            error = pivoterTargetPosition - pivoter.getCurrentPosition();

            derivative = (error - lastError) / timer.seconds();
            integralSum = integralSum + (error * timer.seconds());

            out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            pivoter.setPower(out);

            lastError = error;

            timer.reset();
        }
    }

    private void turatie(){
        turatie.setPower(0.6);//la duratie a sa inceapa lent(adica o sa ruleze contant fara power 0) si cand e bila sigur pregatita power 1
        if(slotIndex == 3){
            turatie.setPower(1.0);
        }
    }

    //de pus while
}