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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.opencv.core.Mat;

//sky dog change
@Disabled
@TeleOp
@Config
public class FSF_Arm extends OpMode{
    private PIDController controller;

    private ElapsedTime looptime = new ElapsedTime();
    private ElapsedTime armtime = new ElapsedTime();

    double offset = -200 +360;

    double time;

    double angle;

    double prevPos;

    double prevVel;

    double armVel;

    double armAccel;

    public static double maxAccel = 3;

    public static double maxVel = 35;

    public double distance = 0;
    // Arm Up constants

    // p = 0.0015, i = 0, d = 0.00007;
    // f = 0.23;
    // k = 0.00001;

    // Arm Down Constants
    // p = 0.005, i = 0, d = 0.002
    // f = 0.12
    // k = 0.00001

    public static double p = 0.0003, v = 0.001 , a = 0.0;


  //  public static double p = 0.0025, i = 0, d = 0.00008;
  //  public static double p = 0.01, i = 0, d = 0.0008;

    public static double f = -0.04;

   // public static double f = -0.05;  //0.12 also good

    public static double k = 0.0003;// the peak power is about 0.7 without p .
    public static int target = 600;
    public static int veltarget = 40;
    public static int acceltarget = 0;



    public static int targetslide = 1000;

    //private final double ticks_in_degree = 5281.1/360;
    private final double ticks_in_degree = 8192.0/360;

    private DcMotorEx  Arm_right;
    private DcMotorEx  Arm_left;


    private DcMotorEx Slide_bot;
    private DcMotorEx Slide_top;

    public AnalogInput Arm_encoder;

    public double motion_profile(double maxaccel, double maxvel, double dist, double t){
        double acceldt = maxvel/maxaccel;
        double half_dist =  dist/2;

        double acceldist = 0.5*maxaccel* (Math.pow(acceldt,2));

        if (acceldist > half_dist){
            acceldt = Math.sqrt(half_dist/ (0.5*maxaccel));
        }
        acceldist = 0.5 * maxaccel * (Math.pow(acceldt,2));

        maxvel = maxaccel * acceldt;
        double decceldt = acceldt;

        double cruisedist = dist - 2*acceldist;
        double cruisedt = cruisedist/maxvel;

        double decceltime = acceldt + cruisedt;

        double totaldt = acceldt + cruisedt + decceldt;
        if (t > totaldt){
            return dist;
        }

        if (t < acceldt){
            return 0.5 * maxaccel * Math.pow(t,2);
        }

        else if (t < decceltime){
            acceldist = 0.5 * maxaccel * Math.pow(acceldt,2);
            double cruisecurrentdist = t -acceldt;
            return acceldist + maxvel * cruisecurrentdist;
        }
        else{
            acceldist = 0.5*maxaccel * Math.pow(acceldt,2);
            cruisedist = maxvel * cruisedt;
            decceltime = t - decceltime;
            return acceldist + cruisedist + maxvel * decceltime - 0.5*maxaccel * Math.pow(decceltime,2);
        }



    }


    public final void pause(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    @Override
    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        armRotate = hardwareMap.get(DcMotorEx.class, "armRotate");
//        armRotateLeft = hardwareMap.get(DcMotorEx.class, "armRotateLeft");
        Arm_right = hardwareMap.get(DcMotorEx.class, "Arm_right");
        Arm_left = hardwareMap.get(DcMotorEx.class, "Arm_left");

        Slide_bot = hardwareMap.get(DcMotorEx.class, "Slide_bot");
        Slide_top = hardwareMap.get(DcMotorEx.class, "Slide_top");

        Arm_encoder= hardwareMap.get(AnalogInput.class, "Arm_encoder");

        Arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Slide_bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide_bot.setTargetPosition(0);
        Slide_bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide_bot.setVelocity(0);

        Slide_top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Slide_top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide_top.setTargetPosition(0);
        Slide_top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide_top.setVelocity(0);



        Arm_right.setPower(0);
        Arm_left.setPower(0);

        armtime.reset();







    }
    @Override
    public void loop(){


        if (gamepad1.triangle ){
            Slide_bot.setTargetPosition(-targetslide);
            Slide_top.setTargetPosition(-targetslide);

            Slide_bot.setVelocity(700);
            Slide_top.setVelocity(700);

            pause(2000);

        }






        angle = 360 - ((Arm_encoder.getVoltage() / 3.2 * 360 + offset) % 360);
        if (angle < 360 && angle > 300) angle-=360;

        double armPos = (angle * 8192.0/360);// negative to change the vaule for easy understanding;

        time = looptime.milliseconds();

        armVel = (armPos - prevPos)/time;

        armAccel = (armVel -  prevVel)/time;


        int slidePos = -Slide_top.getCurrentPosition();

        double posError = target - armPos;
        double velError = veltarget - armVel;
        double accelError = acceltarget - armAccel;
      //  slidePos = 0;
     //   if (Math.abs(armPos-target)<400) d=0.0008; else d=0.002;
        double pid = posError * p + velError * v + accelError*a;
        double ff = Math.cos(Math.toRadians(angle)) * (f +k*slidePos) ;  // target
        double power = pid + ff;

        Arm_left.setPower(-power);
        Arm_right.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("time", armtime.seconds());
        telemetry.addData("vel", armVel);
        telemetry.addData("accel", armAccel);
        telemetry.addData("target", target);
        telemetry.addData("Current Angle", angle);
        telemetry.addData("FF power", ff);
        telemetry.addData("PID power", pid);
        telemetry.addData("Total Power", power);
        telemetry.addData("slide pos", slidePos);

        telemetry.addData("Current right", Arm_right.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Current left", Arm_left.getCurrent(CurrentUnit.AMPS));
        telemetry.update();

        prevPos = armPos;

        prevVel = armVel;

        looptime.reset();

    }
}
