package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

// Claw 0.25 open
// Claw 0.65 closed
//Specimen intake
//
//Intake handle (left right) 0.47, 0.55
// Arm rot degree 215

//Specimen outtake
//p = 0.012
//Intake handle (left right) 0.93 0.50
// Arm rot degree 25


@TeleOp
@Config
public class PIDF_ArmOffA extends OpMode{

    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx  Leftarm;
    private DcMotorEx  Rightarm;



    private Servo Lefthandle;
    private Servo Righthandle;

    private Servo Claw;

    public AnalogInput Armencoder;


    private PIDController controller;

    public double arm_pos = 90;

    double power;

    double handleleft = 0.61;
    double handleright = 0.77;


    public enum State {
        IDLE,
        SAMPLEINTAKE,
        SPECINTAKE,
        SPECOUTTAKE,

    }



    // debug stuff
    double offset = -86 + 360;// -200+360

    double angle = 0;

    public static double p = 0.0075, i = 0, d = 0.0002;

    public static double leftdiffy = 0.5, rightdiffy = 0.5, clawpos = 0.5;

    double pid;
    double ff;

    public static double f = 0.04;


    public static double k = 0.0003;// the peak power is about 0.7 without p .
    public static int target = 180;
    public static int deadband = 1;


    //booleans

    boolean idleflag = true;








    State state = State.IDLE;
    @Override
    public void init(){
        initsetup();

    }
    @Override
    public void loop(){


        Claw.setPosition(clawpos);

//        switch (state){
//            case IDLE:
//
//                if (idleflag){
//                    p = 0.0075;
//                    target = 215;
//                    Claw.setPosition(0.25);
//                    Lefthandle.setPosition(0.1);
//                    Righthandle.setPosition(0.92);
//                    // linear slide idle pos
//                    // slide handle idle pos
//                    idleflag = false;
//
//                }
//
//                if (gamepad2.left_bumper){
//                    Lefthandle.setPosition(0.46);
//                    Righthandle.setPosition(0.56);
//                    p = 0.0075;
//                    target = 213;
//                    Claw.setPosition(0.25);
//
//                    state = State.SPECINTAKE;
//                    break;
//                }
//
//                if (gamepad2.right_bumper){
//                    // extend arm into submersible to get ready for intake
//                    // ready for sample intake
//                    state = State.SAMPLEINTAKE;
//                    break;
//                }
//                break;
//            case SAMPLEINTAKE:
//                break;
//                // linear slide extend and intake
//                //make sure to add breakout condition
//            case SPECINTAKE:
//                if (gamepad1.right_bumper){
//                    Claw.setPosition(0.65);
//                    delay(250);
//                    moveStraight(0.65);
//                    p = 0.012;
//                    target = 23;
//                    delay(200);
//                    Lefthandle.setPosition(0.93);
//                    Righthandle.setPosition(0.5);
//                    moveStraight(0);
//                    state = State.SPECOUTTAKE;
//                    break;
//
//                }
//                break;
//            case SPECOUTTAKE:
//
//
//                if (gamepad1.right_bumper){
//                    idleflag = true;
//                    Claw.setPosition(0.25);
//                    moveStraight(-0.7);
//                    delay(300);
//                    state = State.IDLE;
//                    break;
//                }
//
//                if (gamepad1.left_bumper){
//                    Lefthandle.setPosition(0.46);
//                    Righthandle.setPosition(0.56);
//                    p = 0.0075;
//                    target = 213;
//                    Claw.setPosition(0.25);
//
//                    state = State.SPECINTAKE;
//                    break;
//
//                }
//                break;
//        }
//
//        robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, 1.0);
//
//        armrotatePIDF();
//        telemetry.addData("target", target);
//        telemetry.addData("Current Angle", angle);
//        telemetry.addData("Total Power", power);
//
//        telemetry.update();

    }

    private double square_root(double input) {
        if (input >= 0) {
            return Math.sqrt(input);
        } else {
            return -1 * Math.sqrt(Math.abs(input));

        }
    }

    protected void robot_centric(double iy, double ix, double irx, double ratio) {
        double y = -iy;
        double x = ix * 1.1; // Counteract imperfect strafing
        double rx = irx; // 0.75
        double drivinginput= Math.abs(y) + Math.abs(x) + Math.abs(rx);
        double denominator = Math.max(drivinginput, 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower * ratio);
        rightFront.setPower(frontRightPower * ratio);
        leftBack.setPower(backLeftPower * ratio);
        rightBack.setPower(backRightPower * ratio);


    }

    public final void pause(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void delay(double time)
    {

        int sleepcounter = (int) (time/25);
        for (int i = 0; i < sleepcounter; i++) {
            armrotatePIDF();
            pause(20);
        }
    }

    public double  arm_angle_update()
    {


        angle = (Armencoder.getVoltage() / 3.2 * 360 + offset) % 360;
        if (angle <= 360 && angle >= 250) angle -=360;

        return angle;

    }

    public void armrotatePIDF() {


        arm_pos= arm_angle_update();
        controller.setPID(p,i,d);
        if (Math.abs(angle - target) > deadband) {
            pid = square_root(controller.calculate(angle, target));
        } else {
            pid = 0.0;
        }
        ff = Math.cos(Math.toRadians(angle)) * (f) ;  // target
        power = pid + ff;
        Leftarm.setPower(-power);
        Rightarm.setPower(power);


    }

    public void initsetup(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        armRotate = hardwareMap.get(DcMotorEx.class, "armRotate");
//        armRotateLeft = hardwareMap.get(DcMotorEx.class, "armRotateLeft");
        Rightarm = hardwareMap.get(DcMotorEx.class, "Rightarm");
        Leftarm = hardwareMap.get(DcMotorEx.class, "Leftarm");

        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");


        Armencoder= hardwareMap.get(AnalogInput.class, "Armencoder");

        Lefthandle = hardwareMap.get(Servo.class,"Lefthandle");
        Righthandle = hardwareMap.get(Servo.class,"Righthandle");

        Claw = hardwareMap.get(Servo.class,"Claw");


        Leftarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Leftarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Rightarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Leftarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rightarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void moveStraight (double pow){
        leftFront.setPower(pow);
        leftBack.setPower(pow);
        rightFront.setPower(pow);
        rightBack.setPower(pow);
    }

}
