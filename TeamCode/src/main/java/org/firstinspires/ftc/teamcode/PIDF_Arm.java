package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
@Config
public class PIDF_Arm extends OpMode{
    private PIDController controller;

    double handleleft = 0.24;
    double handleright = 0.36;


    double offset = -200 +360;

    double angle;

    // Arm Up constants

    // p = 0.0015, i = 0, d = 0.00007;
    // f = 0.23;
    // k = 0.00001;

    // Arm Down Constants
    // p = 0.005, i = 0, d = 0.002
    // f = 0.12
    // k = 0.00001

    public static double p = 0, i = 0, d = 0;


  //  public static double p = 0.0025, i = 0, d = 0.00008;
  //  public static double p = 0.01, i = 0, d = 0.0008;

    public static double f = -0.04;

   // public static double f = -0.05;  //0.12 also good

    public static double k = 0.0003;// the peak power is about 0.7 without p .
    public static int target = 0;

    public static int targetslide = 1000;

    //private final double ticks_in_degree = 5281.1/360;
    private final double ticks_in_degree = 8192.0/360;

    private DcMotorEx  Arm_right;
    private DcMotorEx  Arm_left;


    private DcMotorEx Slide_bot;
    private DcMotorEx Slide_top;

    private Servo Left_handle;
    private Servo Right_handle;

    public AnalogInput Arm_encoder;


    public final void pause(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    @Override
    public void init(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        armRotate = hardwareMap.get(DcMotorEx.class, "armRotate");
//        armRotateLeft = hardwareMap.get(DcMotorEx.class, "armRotateLeft");
        Arm_right = hardwareMap.get(DcMotorEx.class, "Arm_right");
        Arm_left = hardwareMap.get(DcMotorEx.class, "Arm_left");

        Slide_bot = hardwareMap.get(DcMotorEx.class, "Slide_bot");
        Slide_top = hardwareMap.get(DcMotorEx.class, "Slide_top");

        Arm_encoder= hardwareMap.get(AnalogInput.class, "Arm_encoder");

        Left_handle = hardwareMap.get(Servo.class,"Left_handle");
        Right_handle = hardwareMap.get(Servo.class,"Right_handle");


        Arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        Slide_bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide_bot.setTargetPosition(0);
        Slide_bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide_bot.setVelocity(0);

        Slide_top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Slide_bot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Slide_top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide_top.setTargetPosition(0);
        Slide_top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide_top.setVelocity(0);

        Left_handle.setPosition(handleleft);
        Right_handle.setPosition(handleright);



        Arm_right.setPower(0);
        Arm_left.setPower(0);



    }
    @Override
    public void loop(){

        if (gamepad1.triangle ){
            Slide_bot.setTargetPosition(-targetslide);
            Slide_top.setTargetPosition(-targetslide);

            Slide_bot.setVelocity(700);
            Slide_top.setVelocity(700);


        }
//
//        if (gamepad1.dpad_up){
//            target = 750;
//            p =0.0004;
////            d = 0.00001;
//        }
//
//        if (gamepad1.dpad_down){
//            target = 0;
//            p =0.0003;
////            d = 0.00001;
//        }



        controller.setPID(p,i,d);

        angle = 360 - ((Arm_encoder.getVoltage() / 3.2 * 360 + offset) % 360);
        if (angle < 360 && angle > 300) angle-=360;

        int armPos = (int) (angle * 8192.0/360);// negative to change the vaule for easy understanding;
//        int slidePos = -Slide_bot.getCurrentPosition();
        int slidePos = -Slide_top.getCurrentPosition();
      //  slidePos = 0;
     //   if (Math.abs(armPos-target)<400) d=0.0008; else d=0.002;
        double pid = controller.calculate(armPos,target);
        double ff = Math.cos(Math.toRadians(angle)) * (f +k*slidePos) ;  // target
        double power = pid + ff;
        Arm_left.setPower(-power);
        Arm_right.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.addData("Current Angle", angle);
        telemetry.addData("FF power", ff);
        telemetry.addData("PID power", pid);
        telemetry.addData("Total Power", power);
        telemetry.addData("slide pos", slidePos);
        telemetry.addData("slide bot pos", Slide_bot.getCurrentPosition());

        telemetry.addData("Current right", Arm_right.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Current left", Arm_left.getCurrent(CurrentUnit.AMPS));

        telemetry.addData("Current slide top", Slide_top.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("current slide bottom", Slide_bot.getCurrent(CurrentUnit.AMPS));
        telemetry.update();

    }
}
