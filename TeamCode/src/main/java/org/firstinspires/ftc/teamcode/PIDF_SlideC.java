package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//183


@TeleOp
@Config
public class PIDF_SlideC extends OpMode {

    private PIDController controller;

    double Slide_pos = 0;


    double handleleft = 0.46;
    double handleright = 0.54;


    double offset = 38;// -200+360

    double angle;
    double raw_angle;

    public static double p = 0.007, i = 0, d = 0.0002;


    //  public static double p = 0.0025, i = 0, d = 0.00008;
    //  public static double p = 0.01, i = 0, d = 0.0008;

    public static double f = 0.03;

    // public static double f = -0.05;  //0.12 also good

    public static double k = 0.0003;// the peak power is about 0.7 without p .
    public static int target = 0;

    public static int deadband = 20;

    public static int targetslide = 1000;

    //private final double ticks_in_degree = 5281.1/360;
    private final double ticks_in_degree = 8192.0 / 360;

    private DcMotorEx Arm_right;
    private DcMotorEx Arm_left;

    private DcMotorEx Slide_encoder;


    private DcMotorEx Slide_bot;
    private DcMotorEx Slide_top;

    private Servo Left_handle;
    private Servo Right_handle;

    public AnalogInput Arm_encoder;

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
        Arm_right = hardwareMap.get(DcMotorEx.class, "Arm_right");
        Arm_left = hardwareMap.get(DcMotorEx.class, "Arm_left");

        Slide_bot = hardwareMap.get(DcMotorEx.class, "Slide_bot");
        Slide_top = hardwareMap.get(DcMotorEx.class, "Slide_top");

        Arm_encoder = hardwareMap.get(AnalogInput.class, "Arm_encoder");

        Slide_encoder = hardwareMap.get(DcMotorEx.class, "leftFront");

        Left_handle = hardwareMap.get(Servo.class, "Left_handle");
        Right_handle = hardwareMap.get(Servo.class, "Right_handle");


        Arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Slide_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        Slide_bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide_bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Slide_top.setDirection(DcMotorSimple.Direction.REVERSE);
        Slide_bot.setDirection(DcMotorSimple.Direction.REVERSE);


        Slide_top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Slide_bot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        Slide_top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide_top.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 0.615
        //0.775

        Left_handle.setPosition(handleleft);
        Right_handle.setPosition(handleright);


        Arm_right.setPower(0);
        Arm_left.setPower(0);


    }

    @Override
    public void loop() {

        Slide_pos = Math.floor(Slide_encoder.getCurrentPosition()/30);

//
        controller.setPID(p, i, d);
//
        raw_angle = Arm_encoder.getVoltage() / 3.2 * 360;
//
        angle = 360 - ((Arm_encoder.getVoltage() / 3.2 * 360 + offset) % 360);
        if (angle < 360 && angle > 330) angle -= 360;
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

        double ff2 = Math.cos(Math.toRadians(angle)) * (-0.04 + k * Slide_pos);  // target

        Slide_top.setPower(power);
        Slide_bot.setPower(power);


        Arm_left.setPower(-ff2);


        Arm_right.setPower(ff2);

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
        telemetry.addData("Arm FF", ff2);
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






