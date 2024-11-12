package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
@Disabled
public class PIDF_Arm extends OpMode{
    private PIDController controller;

    // Arm Up constants

    // p = 0.0015, i = 0, d = 0.00007;
    // f = 0.23;
    // k = 0.00001;

    // Arm Down Constants
    // p = 0.005, i = 0, d = 0.002
    // f = 0.12
    // k = 0.00001

    public static double p = 0.0008, i = 0, d = 0.00009;
  //  public static double p = 0.01, i = 0, d = 0.0008;

    public static double f = 0.12;  //0.12 also good

    public static double k = 0.00002;
    public static int target = 600;

    public static int targetslide = 500;

    //private final double ticks_in_degree = 5281.1/360;
    private final double ticks_in_degree = 8192/360;

    private DcMotorEx  Arm_right;
    private DcMotorEx  Arm_left;


    private DcMotorEx armSlide;


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

        armSlide = hardwareMap.get(DcMotorEx.class, "armSlide");

        Arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setTargetPosition(0);
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlide.setVelocity(0);
        Arm_right.setPower(0);
        Arm_left.setPower(0);

    }
    @Override
    public void loop(){

        if (gamepad1.triangle ){
            armSlide.setTargetPosition(targetslide);
            armSlide.setVelocity(2000);
            pause(2000);

        }



        controller.setPID(p,i,d);
        int armPos =  Arm_right.getCurrentPosition();
        int slidePos = armSlide.getCurrentPosition();
     //   if (Math.abs(armPos-target)<400) d=0.0008; else d=0.002;
        double pid = controller.calculate(armPos,target);
        double ff = Math.cos(Math.toRadians(armPos/ticks_in_degree - 47)) * (f + k*slidePos) ;  // target

        double power = pid + ff;

        Arm_right.setPower(power);
        Arm_left.setPower(-power);
        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.addData("Current Angle", (armPos/ticks_in_degree) - 47 );
        telemetry.addData("FF power", ff);
        telemetry.addData("PID power", pid);
        telemetry.addData("Total Power", power);
       // telemetry.addData("slide pos", slidePos);
        telemetry.update();

    }
}
