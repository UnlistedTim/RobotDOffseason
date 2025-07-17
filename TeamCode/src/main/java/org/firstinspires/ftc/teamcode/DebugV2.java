package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;


import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


//CLAW POS Fully open 0.72, Fully closed 0.93;

@TeleOp(name = "DebugV2", group = "B")

@Config
public class DebugV2 extends LinearOpMode {

    double slidepower = 0.0;

    private PIDController controller;

    public static double p = 0, i = 0, d = 0;

    public static int target = 0;


    //  public static double p = 0.0025, i = 0, d = 0.00008;
    //  public static double p = 0.01, i = 0, d = 0.0008;

    public static double f = -0.04;

//    public static double k = 0.0003;// the peak power is about 0.7 without p .

    double lefthandlepos =  0.5; // 0.59
    double righthandlepos = 0.5; //0.41

    public static double clawpos = 0;

    public static double gearboxpos = 0;

    double clawopen = 0.04;
    double clawclose = 0.48;

    double offset = 38;


    double arm_angle_specintake = 195;

    double lefthandle_specintake = 0.67, righthandle_specintake = 0.35;


    public DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public ColorSensor Intake_color;

    double angle;

//    public static double p = 0.0002, i = 0, d = 0;
//
//
//    //  public static double p = 0.0025, i = 0, d = 0.00008;
//    //  public static double p = 0.01, i = 0, d = 0.0008;
//
//    public static double f = -0.04;

    // public static double f = -0.05;  //0.12 also good

    public static double k = 0.0004;// the peak power is about 0.7 without p .

    public static double leftdiffypos = 0.63;

    public static double rightdiffypos = 0.38;

    public InterpLUT In_LHandle = new InterpLUT();
    public InterpLUT In_RHandle = new InterpLUT();

    public InterpLUT Out_LHandle = new InterpLUT();
    public InterpLUT Out_RHandle= new InterpLUT();



    public  DcMotorEx Arm_right, Arm_left, Slide_top,Slide_bot;
    public Servo Left_handle,Right_handle, Gearbox, Claw;
    public DigitalChannel Arm_touch;
    public VoltageSensor voltageSensor;

    public  DcMotorEx revEncoder;

//    public PIDController controller;

    public DistanceSensor bar_dist;
    public DistanceSensor basket_dist;
    public AnalogInput Arm_encoder;

//    public ColorSensor Claw_color;




//regional

    @Override
    public void runOpMode() {

        controller = new PIDController(p,i,d);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
//        Claw_color=hardwareMap.get(ColorSensor.class, "Claw_color");








//        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
//        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
//                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;


        Arm_right = hardwareMap.get(DcMotorEx.class, "Arm_right");
        revEncoder = hardwareMap.get(DcMotorEx.class,"Arm_right");

        Arm_left = hardwareMap.get(DcMotorEx.class, "Arm_left");
        Slide_bot = hardwareMap.get(DcMotorEx.class, "Slide_bot");
        Slide_top = hardwareMap.get(DcMotorEx.class, "Slide_top");

        Arm_encoder= hardwareMap.get(AnalogInput.class, "Arm_encoder");

//        arm_grab = hardwareMap.get(Servo.class, "arm_grab");
        Right_handle = hardwareMap.get(Servo.class, "Right_handle");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Left_handle = hardwareMap.get(Servo.class, "Left_handle");
        Gearbox = hardwareMap.get(Servo.class, "Gearbox");

//        Intake_color = hardwareMap.get(ColorSensor.class, "Intake_color");
        Arm_touch = hardwareMap.get(DigitalChannel.class,"Arm_touch");

        basket_dist = hardwareMap.get(DistanceSensor.class,"basket_dist");
        bar_dist = hardwareMap.get(DistanceSensor.class,"bar_dist");



        // left 0.61, 0.77
        //  LHandle_correction.add(0,0.655);
        In_LHandle.add(arm_angle_specintake-5,lefthandle_specintake+0.04);
        In_LHandle.add(arm_angle_specintake-4,lefthandle_specintake+0.03);
        In_LHandle.add(arm_angle_specintake-2,lefthandle_specintake+0.015);
        In_LHandle.add(arm_angle_specintake,lefthandle_specintake);
        In_LHandle.add(arm_angle_specintake+4,lefthandle_specintake-0.025);
        In_LHandle.add(arm_angle_specintake+7,lefthandle_specintake-0.045);
        // LHandle_correction.add(350,0.575);
        //   LHandle_correction.add(350,0.4);


        // right 0.77

        //   RHandle_correction.add(0,0.725);
        In_RHandle.add(arm_angle_specintake-5,righthandle_specintake - 0.04);
        In_RHandle.add(arm_angle_specintake-4,righthandle_specintake - 0.03);
        In_RHandle.add(arm_angle_specintake-2,righthandle_specintake - 0.015);
        In_RHandle.add(arm_angle_specintake,righthandle_specintake);
        In_RHandle.add(arm_angle_specintake+4,righthandle_specintake + 0.025);
        In_RHandle.add(arm_angle_specintake+7,righthandle_specintake + 0.045);
        //   RHandle_correction.add(350,0.815);
        //     RHandle_correction.add(350,0.99);

        In_LHandle.createLUT();
        In_RHandle.createLUT();



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.addLine("Wait ! Initializing............. ");
        telemetry.update();

        Arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        Slide_bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Slide_bot.setTargetPosition(0);
//        Slide_bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Slide_bot.setVelocity(0);
//
//        Slide_top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Slide_top.setTargetPosition(0);
//        Slide_top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Slide_top.setVelocity(0);

        Slide_bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Slide_top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Slide_bot.setDirection(DcMotorSimple.Direction.REVERSE);
        Slide_top.setDirection(DcMotorSimple.Direction.REVERSE);

        Slide_bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Slide_top.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        if (isStopRequested()) return;

        telemetry.addLine("Press Start Now!:");
        telemetry.update();

        Gearbox.setPosition(0.05); // 0.05
//        init(2);

        //    rotatetargetPIDF(rotateStart);
        waitForStart();

        Left_handle.setPosition(lefthandlepos);
        Right_handle.setPosition(righthandlepos);
        Claw.setPosition(clawpos);
        Arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Intake_rot.setPosition(0.3);
        //Intake_rot.setPosition(0.3);
        sleep(500);


        while (opModeIsActive()) {

            Claw.setPosition(clawpos);
            angle = 360 - ((Arm_encoder.getVoltage() / 3.2 * 360 + offset) % 360);
            if (angle < 360 && angle > 330) angle-=360;

            controller.setPID(p,i,d);

            int revpos = revEncoder.getCurrentPosition();//todo
           int  slidePos = (int)(revpos/28.407);

            int armPos = (int) (angle * 8192.0/360);// negative to change the vaule for easy understanding;

            double pid = controller.calculate(armPos,target);

            pid = 0;
            double ff = Math.cos(Math.toRadians(angle)) * (f +k*slidePos) ;  // target
            double power = pid + ff;
            Arm_left.setPower(-power);
            Arm_right.setPower(power);

            Gearbox.setPosition(gearboxpos);

//            if (angle >= arm_angle_specintake -5 &&angle <= arm_angle_specintake+7){
//                Left_handle.setPosition(In_LHandle.get(angle));
//                Right_handle.setPosition(In_RHandle.get(angle));
//            }

                Left_handle.setPosition(leftdiffypos);

                Right_handle.setPosition(rightdiffypos);



           // Intake_rot.setPosition(0);


//            if (gamepad2.triangle) {armslidePos+= armslideStep; rbg.timer(0,0);}
//            if (gamepad2.cross) {rbg.armslidePos -= rbg.armslideStep;; rbg.timer(0,0);}
//            if (gamepad2.square) {rbg.armrotatePos -=rbg.armrotateStep; rbg.timer(0,0);}
//            if (gamepad2.square) {rbg.armrotatePos-= ; rbg.timer(0,0);}
//            if (gamepad2.circle) {rbg.armrotatePos += rbg.armrotateStep;; rbg.timer(0,0);}


//

            telemetry.addData("armlinerslide", slidePos);
            telemetry.addData("Slide power", slidepower);
            telemetry.addData("armrotate position", angle);
//
            telemetry.addData("left handle pos", leftdiffypos);
            telemetry.addData("right handle pos", rightdiffypos);
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








