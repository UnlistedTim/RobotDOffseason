package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;


import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;


//CLAW POS Fully open 0.72, Fully closed 0.93;

@TeleOp(name = "DebugVD", group = "A")
public class DebugVD extends LinearOpMode {

    double slidepower = 0.0;

    double lefthandlepos =  0.5; // 0.59
    double righthandlepos = 0.5; //0.41

    double clawpos = 0;

    double clawopen = 0.04;
    double clawclose = 0.48;

    double offset = 38;




    double angle;

    public static double p = 0.0002, i = 0, d = 0;


    //  public static double p = 0.0025, i = 0, d = 0.00008;
    //  public static double p = 0.01, i = 0, d = 0.0008;

    public static double f = -0.04;

    // public static double f = -0.05;  //0.12 also good

    public static double k = 0.0003;// the peak power is about 0.7 without p .



    public  DcMotorEx leftFront, leftBack, rightBack, rightFront;
//    public  DcMotorEx revEncoder;

    public DcMotorEx backBotSlide, backTopSlide, frontBotSlide, frontTopSlide;


    public  Servo rightLink,rightPivot, Claw, rightGearbox;

    public Servo leftLink, leftPivot, leftGearbox;

    public Servo extendoArm, Cam, extendoHandle, rightDiffy, leftDiffy;
    public VoltageSensor voltageSensor;

    public PIDController controller;


//    public ColorSensor Claw_color;




//regional

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");


//        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
//        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
//                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;


        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

//        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
//        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
//                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;


        backBotSlide = hardwareMap.get(DcMotorEx.class, "backBotSlide");
        backTopSlide = hardwareMap.get(DcMotorEx.class, "backTopSlide");
        frontBotSlide = hardwareMap.get(DcMotorEx.class, "frontBotSlide");
        frontTopSlide = hardwareMap.get(DcMotorEx.class, "frontTopSlide");


        rightLink = hardwareMap.get(Servo.class, "rightLink");
        Claw = hardwareMap.get(Servo.class, "Claw");
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");
        rightGearbox = hardwareMap.get(Servo.class, "rightGearbox");

        leftLink = hardwareMap.get(Servo.class, "leftLink");
        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        leftGearbox = hardwareMap.get(Servo.class, "leftGearbox");



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.addLine("Wait ! Initializing............. ");
        telemetry.update();


        voltageSensor = hardwareMap.voltageSensor.iterator().next();




        backBotSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backBotSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backTopSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backTopSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontBotSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontBotSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontTopSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontTopSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);





        if (isStopRequested()) return;

        telemetry.addLine("Press Start Now!:");
        telemetry.update();

        Gearbox.setPosition(0.95); // 0.05
//        init(2);

        //    rotatetargetPIDF(rotateStart);
        waitForStart();


        // Intake_rot.setPosition(0.3);
        //Intake_rot.setPosition(0.3);
        sleep(500);


        while (opModeIsActive()) {
            angle = 360 - ((Arm_encoder.getVoltage() / 3.2 * 360 + offset) % 360);
            if (angle < 360 && angle > 330) angle-=360;
           // Intake_rot.setPosition(0);


//            if (gamepad2.triangle) {armslidePos+= armslideStep; rbg.timer(0,0);}
//            if (gamepad2.cross) {rbg.armslidePos -= rbg.armslideStep;; rbg.timer(0,0);}
//            if (gamepad2.square) {rbg.armrotatePos -=rbg.armrotateStep; rbg.timer(0,0);}
//            if (gamepad2.square) {rbg.armrotatePos-= ; rbg.timer(0,0);}
//            if (gamepad2.circle) {rbg.armrotatePos += rbg.armrotateStep;; rbg.timer(0,0);}
            if (gamepad2.dpad_up) {
                if (lefthandlepos <= 0.996 && righthandlepos >=0.004){
                    lefthandlepos+=0.005;
                    righthandlepos-=0.005;
                    Left_handle.setPosition(lefthandlepos);
                    Right_handle.setPosition(righthandlepos);
                    sleep(300);
                }



            }
            if (gamepad2.dpad_down) {
                if (lefthandlepos >= 0.004 && righthandlepos <=0.996) {
                    lefthandlepos-=0.005;
                    righthandlepos+=0.005;
                    Left_handle.setPosition(lefthandlepos);
                    Right_handle.setPosition(righthandlepos);
                    sleep(300);
                }


            }
            if (gamepad2.dpad_left) {
                if (righthandlepos >= 0.004 && lefthandlepos >=0.004){
                    lefthandlepos-=0.005;
                    righthandlepos-=0.005;
                    Left_handle.setPosition(lefthandlepos);
                    Right_handle.setPosition(righthandlepos);

                    sleep(300);
                }

            }
            if (gamepad2.dpad_right) {
                if (righthandlepos <= 0.996 && lefthandlepos <=0.996){
                    lefthandlepos+=0.005;
                    righthandlepos+=0.005;
                    Left_handle.setPosition(lefthandlepos);
                    Right_handle.setPosition(righthandlepos);
                    sleep(300);
                }

            }
            if (gamepad2.triangle) {

                Claw.setPosition(clawclose);
                clawpos = clawclose;


            }
            if (gamepad2.cross) {
                Claw.setPosition(clawopen);
                clawpos = clawopen;
            }

            if (gamepad1.dpad_up){
                slidepower +=0.02;
                Slide_top.setPower(slidepower);
                Slide_bot.setPower(slidepower);
                sleep(200);

            }

            if (gamepad1.dpad_down){
                slidepower -=0.02;
                Slide_top.setPower(slidepower);
                Slide_bot.setPower(slidepower);
                sleep(200);

            }



            if (gamepad2.circle) {
            }
            if (gamepad2.square) {
            }
//            if(gamepad1.dpad_up) {
//                if (clawpos <= 0.98){
//                    clawpos+=0.02;
//                    Claw.setPosition(clawpos);
//                    sleep(300);
//                }
////                tar=tar+100;
////                linearslide(tar,2700);
//
//            }
//            if(gamepad1.dpad_down)
//            {
//                if (clawpos >= 0.02){
//                    clawpos-=0.02;
//                    Claw.setPosition(clawpos);
//                    sleep(300);
//                }
//
////                tar=tar-100;
////                rbg.linearslide(tar,2700);
//
//            }


//            if (gamepad1.dpad_up){
//
//            }
//
//            if (gamepad1.dpad_down){
//
//            }

//

            telemetry.addData("armlinerslide", Slide_top.getCurrentPosition());
            telemetry.addData("Slide power", slidepower);
            telemetry.addData("armrotate position", angle);
//
            telemetry.addData("left handle pos", lefthandlepos);
            telemetry.addData("right handle pos", righthandlepos);
//
//            telemetry.addData("CLaw pos", clawpos);
//
//
//
////
//           telemetry.addData("gerbox", Gearbox.getPosition());
//
//           telemetry.addData("Color sensor red", Claw_color.red());
//            telemetry.addData("Color sensor green", Claw_color.red());
//            telemetry.addData("Color sensor blue", Claw_color.blue());
//            telemetry.addLine("gamepad2  dapad down/up handle        dpad left/right hadle ort");
//            telemetry.addLine("gamepad2  triangle clawcolse        cross claw open");
//            telemetry.addLine("gamepad1  dapd up  +        dpad downn-");
//            telemetry.addData("Bar distance", bar_dist.getDistance(DistanceUnit.MM));
//            telemetry.addData("Basket distance", rbg.basket_dist.getDistance(DistanceUnit.MM));
      //      telemetry.addLine("gamepad2 right/lefit bumper   handle rot");

//            telemetry.addData("Intake rot",intakerotpose);
            telemetry.update();
          //  update_flag=false;








            }
        }

}








