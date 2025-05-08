package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//183


@TeleOp
@Config
public class PIDF_SlideD extends OpMode {

    private PIDController controller;

    double Slide_pos = 0;



    double offset = 38;// -200+360

    double angle = 90;
    double raw_angle;

    public static double p = 0.004, i = 0, d = 0;


    //  public static double p = 0.0025, i = 0, d = 0.00008;
    //  public static double p = 0.01, i = 0, d = 0.0008;

    public static double f = 0.05;

    // public static double f = -0.05;  //0.12 also good

    public static double k = 0.0003;// the peak power is about 0.7 without p .
    public static int target = 0;

    public static int deadband = 20;

    public static int targetslide = 1000;

    //private final double ticks_in_degree = 5281.1/360;

    public DcMotorEx backBotSlide, backTopSlide, frontBotSlide, frontTopSlide;

    private DcMotorEx Slide_enencoder;




    double pid;
    double ff;

    double power;


    public final void pause(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        armRotate = hardwareMap.get(DcMotorEx.class, "armRotate");
//        armRotateLeft = hardwareMap.get(DcMotorEx.class, "armRotateLeft");


//        Arm_encoder = hardwareMap.get(AnalogInput.class, "Arm_encoder");

//        Slide_enencoder = hardwareMap.get(DcMotorEx.class, "leftFront");

//        Left_handle = hardwareMap.get(Servo.class, "Left_handle");
//        Right_handle = hardwareMap.get(Servo.class, "Right_handle");

        backBotSlide = hardwareMap.get(DcMotorEx.class, "backBotSlide");
        backTopSlide = hardwareMap.get(DcMotorEx.class, "backTopSlide");
        frontBotSlide = hardwareMap.get(DcMotorEx.class, "frontBotSlide");
        frontTopSlide = hardwareMap.get(DcMotorEx.class, "frontTopSlide");



        backBotSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backBotSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backTopSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backTopSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontBotSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontBotSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontTopSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontTopSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        backBotSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        backTopSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        frontBotSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        frontTopSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        backBotSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backTopSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontBotSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontTopSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);





        // 0.615
        //0.775


    }

    @Override
    public void loop() {

        Slide_pos = backBotSlide.getCurrentPosition();


//
        controller.setPID(p, i, d);
//
//        int armPos = (int) (angle * 8192.0/360);// negative to change the vaule for easy understanding;
////        int slidePos = -Slide_bot.getCurrentPosition();
//        int slidePos = -Slide_top.getCurrentPosition();
//      //  slidePos = 0;
//     //   if (Math.abs(armPos-target)<400) d=0.0008; else d=0.002;

        if (Math.abs(Slide_pos - target) > deadband) {
            pid = square_root(controller.calculate(Slide_pos, target));
        } else {
            pid = 0.0;
        }

        double ff = Math.sin(Math.toRadians(angle)) * f;
        double power = pid + ff;

//        double ff2 = Math.cos(Math.toRadians(angle)) * (-0.04 + k * Slide_pos);  // target

        backBotSlide.setPower(power);
        backTopSlide.setPower(power);
        frontBotSlide.setPower(power);
        frontTopSlide.setPower(power);



//        telemetry.addData("pos", armPos);
//        telemetry.addData("target", target);
//        telemetry.addData("Current Angle", angle);
//        telemetry.addData("Raw Angle", raw_angle);
//        telemetry.addData("FF power", ff);
//        telemetry.addData("PID power", pid);
//        telemetry.addData("Total Power", power);
//        telemetry.addData("slide pos", slidePos);
//        telemetry.addData("slide bot pos", Slide_bot.getCurrentPosition());

        telemetry.addData("Slide encoder Pos", Slide_pos);
        telemetry.addData("Arm angle", angle);
        telemetry.addData("Power", power);
        telemetry.addData("FF power", ff);
        telemetry.addData("PID", pid);
        telemetry.update();

    }

    private double square_root(double input) {
        if (input >= 0) {
            return Math.sqrt(input);
        } else {
            return -1 * Math.sqrt(Math.abs(input));

        }
    }
}






