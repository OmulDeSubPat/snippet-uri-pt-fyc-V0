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
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.util.ElapsedTime;
    import com.sun.tools.javac.processing.PrintingProcessor;

    import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

    @TeleOp(name = "&TeleOpMain")
    public class TeleOpMain extends LinearOpMode {

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
        double prev_t = 0;
        ColorSensor colorsensorSLot1;
        ColorSensor colorsensorSLot2;
        ColorSensor colorsensorSLot3;
        DcMotor intake;
        DcMotorEx tureta;
        Servo ejector;
        DcMotor front_left;
        DcMotor front_right;
        DcMotor back_left;
        DcMotor back_right;
        DcMotorEx flywheel;
        Servo spinnerCLose;
        Servo spinnerFar;


        boolean step1Done = false;
        boolean step2Done = false;
        boolean step3Done = false;
        boolean step4Done = false;
        boolean step5Done = false;
        boolean step6Done = false;
        boolean step7Done = false;
        boolean step8Done = false;
        boolean step9Done = false;
        boolean step10Done = false;
        boolean step11Done = false;

        Limelight3A limelight;
        IMU imu;

        double DEG_PER_TICK = 360.0 / 383.6;
        boolean flywheelOn = false;
        boolean intakeMode = false;
        boolean intakeReverse = false;
        boolean outtakeMode = false;
        private ElapsedTime spinnerTimeout = new ElapsedTime();
        private ElapsedTime outtakeTimeout = new ElapsedTime();
        double ejectorDown = 0.285;
        double ejectorUp = 0.005;
        PinpointLocalizer pinpoint;
        Pose pose;
        double CoordX, CoordY, header;
        double Posspinner = 0;
        double PosspinnerMin = 0;
        double PosspinnerMax = 0.95;
        int ballsLoaded = 0;

        static final double FLYWHEEL_TICKS_PER_REV = 384.5;
        static final double TARGET_RPM = 180;

        double flywheelPowerHigh = 0.6;
        double flywheelPowerLow = 0.5;

        double flywheelTolerance = 20; // RPM


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
            tureta = hardwareMap.get(DcMotorEx.class, "tureta");
            flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

            tureta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            tureta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            tureta.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        private void InitServo() {
            ejector = hardwareMap.get(Servo.class, "ejector");//0.285 down 0.005 up
            spinnerFar = hardwareMap.get(Servo.class, "SpinnerFar");
            spinnerCLose = hardwareMap.get(Servo.class, "SpinnerClose");
            ejector.setPosition(0.285);
            spinnerFar.setPosition(0);
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
            pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
            Pose startPos = new Pose(0, 0, 0);
            pinpoint.setStartPose(startPos);
        }


        private void SetWheelsPower() {
            double left_x = gamepad1.left_stick_x;
            double left_y = -gamepad1.left_stick_y; // forward is negative
            double right_x = gamepad1.right_stick_x;

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


        private int smekerie(ColorSensor colorSensor) {
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

        private int CuloareFinala(ColorSensor sensor, int[] last5, int index) {
            last5[index] = smekerie(sensor);

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
            Color1 = CuloareFinala(colorsensorSLot1, last5Sensor1, indexSensor1);
            indexSensor1 = (indexSensor1 + 1) % 5;
            Color2 = CuloareFinala(colorsensorSLot2, last5Sensor2, indexSensor2);
            indexSensor2 = (indexSensor2 + 1) % 5;
            Color3 = CuloareFinala(colorsensorSLot3, last5Sensor3, indexSensor3);
            indexSensor3 = (indexSensor3 + 1) % 5;
        }

        private void flywheelLogic() {
            if (gamepad1.shareWasPressed()) {
                flywheelOn = !flywheelOn;
            }

            if (flywheelOn) {
                double rpm = getFlywheelRPM();
                double power = calculateBang(TARGET_RPM, rpm);
                flywheel.setPower(power);

                telemetry.addData("Flywheel RPM", rpm);
                telemetry.addData("Flywheel Power", power);
            } else {
                flywheel.setPower(0);
            }
        }


        private void servoLogic() {
            if (gamepad1.optionsWasReleased()) {
                ejector.setPosition(0.285);
            } else if (gamepad1.optionsWasPressed()) {
                ejector.setPosition(0.005);
            }
            if (gamepad1.touchpadWasPressed())
                Posspinner = 0;
            //0.19=60 de grade
            if (gamepad1.dpadRightWasPressed()) Posspinner = Posspinner + 0.19;
            if (gamepad1.dpadLeftWasPressed()) Posspinner = Posspinner - 0.19;
        }

        private boolean spinnerFull() {
            if (Color1 != 0 && Color2 != 0 && Color3 != 0) return true;
            else return false;
        }

        private double getFlywheelRPM() {
            return flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
        }

        private double calculateBang(double targetRPM, double currentRPM) {

            if (currentRPM < targetRPM - flywheelTolerance) {
                return flywheelPowerHigh;   // accelerează
            } else if (currentRPM > targetRPM + flywheelTolerance) {
                return flywheelPowerLow;    // coast
            } else {
                return flywheelPowerLow;    // menține
            }
        }


        private void colorDrivenSpinnerLogic() {

            if (spinnerFull()) return;
            detectedBalls = 0;
            if (Color1 != 0) detectedBalls++;
            if (Color2 != 0) detectedBalls++;
            if (Color3 != 0) detectedBalls++;

            if (Color1!=0 && spinnerTimeout.milliseconds() >= 500) {

                switch (detectedBalls) {
                    case 1:
                        Posspinner = 0.19;
                        break;
                    case 2:
                        Posspinner = 0.38;
                        break;
                    case 3:
                        Posspinner = 0.38+0.19;
                        break;
                }

                spinnerTimeout.reset();
            }
        }





        /*
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
        */


        private void updateTelemetry() {
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
            telemetry.addData("timp_intake", spinnerTimeout.time());
            telemetry.addData("timp_outtake", outtakeTimeout.time());
            telemetry.addData("x", CoordX);
            telemetry.addData("y", CoordY);
            telemetry.addData("heading", header);
            telemetry.addData("unghiSPinner", spinnerFar.getPosition());
            telemetry.addData("close", spinnerCLose.getPosition());
            telemetry.addData("balls",detectedBalls);
            telemetry.update();
        }


        private void runOuttake() {
            intake.setPower(-1);
            final int EJECTOR_UP_DELAY = 300;
            final int EJECTOR_DOWN_DELAY = 300;
            final int SPINNER_SLOT_CHANGE_DELAY = 300;
            final int INITIAL_DELAY = 1200;

            slots[0] = Color1;
            slots[1] = Color2;
            slots[2] = Color3;

            Color1 = 0;
            Color2 = 0;
            Color3 = 0;


            double t = outtakeTimeout.milliseconds();

            if (t - prev_t >= 10 && !step1Done) {
                Posspinner = 0.085;// 0.1
                step1Done = true;
                prev_t = 10;
            }

            if (t >= prev_t + INITIAL_DELAY && !step2Done && step1Done) {
                ejector.setPosition(ejectorUp);
                step2Done = true;
                prev_t += INITIAL_DELAY;
            }

            if (t >= prev_t + EJECTOR_UP_DELAY && !step3Done && step2Done) {
                ejector.setPosition(ejectorDown);
                step3Done = true;
                prev_t += EJECTOR_UP_DELAY;
            }

            if (t >= prev_t + EJECTOR_DOWN_DELAY && !step4Done && step3Done) {
                Posspinner = 0.28; // 0.29
                step4Done = true;
                prev_t += EJECTOR_DOWN_DELAY;
            }

            if (t >= prev_t + SPINNER_SLOT_CHANGE_DELAY && !step5Done && step4Done) {
                ejector.setPosition(ejectorUp);
                step5Done = true;
                prev_t += SPINNER_SLOT_CHANGE_DELAY;
            }

            if (t >= prev_t + EJECTOR_UP_DELAY && !step6Done  && step5Done) {
                ejector.setPosition(ejectorDown);
                step6Done = true;
                prev_t += EJECTOR_UP_DELAY;
            }

            if (t >= prev_t + EJECTOR_DOWN_DELAY && !step7Done && step6Done) {
                Posspinner = 0.46;// 0.48
                step7Done = true;
                prev_t += EJECTOR_DOWN_DELAY;
            }

            if (t >= prev_t + SPINNER_SLOT_CHANGE_DELAY  && !step8Done && step7Done) {
                ejector.setPosition(ejectorUp);
                step8Done = true;
                prev_t += SPINNER_SLOT_CHANGE_DELAY;
            }

            if (t >= prev_t + EJECTOR_UP_DELAY && !step9Done && step8Done) {
                ejector.setPosition(ejectorDown);
                step9Done = true;
                prev_t += EJECTOR_UP_DELAY;
            }
            if (t >= prev_t + EJECTOR_DOWN_DELAY && !step10Done && step9Done) {
                Posspinner = 0;
                step10Done = true;
                outtakeMode = false;
                intakeMode = false;
                ballsLoaded = 0;
                prev_t = 0;
            }
        }


            private void Localizare ()
            {
                pinpoint.update();
                Pose position = pinpoint.getPose();
                CoordX = position.getX();
                CoordY = position.getY();
                header = position.getHeading();

            }


            @Override
            public void runOpMode () {
                InitWheels();
                InitAux();
                InitDc();
                InitLL();
                InitServo();

                waitForStart();

                while (opModeIsActive()) {
                    if (Posspinner >= PosspinnerMin && Posspinner <= PosspinnerMax)
                        spinnerFar.setPosition(Posspinner);
                    servoLogic();
                    updateTelemetry();
                    SetWheelsPower();
                    flywheelLogic();
                    if (gamepad1.crossWasPressed()  && !gamepad1.crossWasReleased()){
                        intake.setPower(1);
                    } else if (gamepad1.crossWasReleased()) {
                        intake.setPower(intakeMode ? -1:0);
                    }
                    if (gamepad1.circleWasPressed()) {
                        intake.setPower(-1);
                        intakeMode = true;
                        outtakeMode = false;
                        ballsLoaded = 0;
                        spinnerFar.setPosition(0);
                    }

//                    if (gamepad1.psWasPressed()) {
//                        intake.setPower(1);
//                    }

                    if (gamepad1.yWasPressed()) {
                        intake.setPower(0);
                    }

                    if (intakeMode && !outtakeMode) {
                        updateCulori();
                        colorDrivenSpinnerLogic();
                    }


                    if (gamepad1.right_trigger > 0.8) {
                        outtakeMode = true;
                        intakeMode = false;
                        intake.setPower(0);
                        outtakeTimeout.reset();

                        step1Done = false;
                        step2Done = false;
                        step3Done = false;
                        step4Done = false;
                        step5Done = false;
                        step6Done = false;
                        step7Done = false;
                        step8Done = false;
                        step9Done = false;
                        step10Done = false;
                    }

                    if (outtakeMode) {
                        runOuttake();
                    }
                    Localizare();
                    idle();
                }
                //ETC

            }
        }

