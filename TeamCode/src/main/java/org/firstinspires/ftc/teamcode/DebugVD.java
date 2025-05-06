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

    double frontslidepowers = 0.0;
    double backslidepowers = 0.0;




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

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        backBotSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        backTopSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        frontBotSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        frontTopSlide.setDirection(DcMotorSimple.Direction.REVERSE);





        if (isStopRequested()) return;

        telemetry.addLine("Press Start Now!:");
        telemetry.update();
//        init(2);

        //    rotatetargetPIDF(rotateStart);
        waitForStart();


        // Intake_rot.setPosition(0.3);
        //Intake_rot.setPosition(0.3);
        sleep(500);


        while (opModeIsActive()) {

            backBotSlide.setPower(backslidepowers);
            backTopSlide.setPower(backslidepowers);

            frontBotSlide.setPower(frontslidepowers);
            frontTopSlide.setPower(frontslidepowers);

            if (gamepad1.dpad_up && frontslidepowers <=0.96){
                frontslidepowers+=0.05;
            }

            if (gamepad1.dpad_down && frontslidepowers >=-0.96){
                frontslidepowers-=0.05;
            }

            if (gamepad1.right_bumper && backslidepowers <=0.96){
                backslidepowers+=0.05;
            }

            if (gamepad1.dpad_left && backslidepowers >=-0.96){
                backslidepowers-=0.05;
            }

            robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x,1.0);

            }

        telemetry.addData("Front slide powers",frontslidepowers);
        telemetry.addData("back slide powers", backslidepowers);


        }

    protected void robot_centric(double iy, double ix, double irx, double ratio) {
        double y = -iy;
        double x = ix * 1.1; // Counteract imperfect strafing
        double rx = irx * 0.75; // 0.75
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower * ratio);
        rightFront.setPower(frontRightPower * ratio);
        leftBack.setPower(backLeftPower * ratio);
        rightBack.setPower(backRightPower * ratio);


    }

}








