package org.firstinspires.ftc.teamcode;

import android.os.Handler;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp

public class AdamTeleopV2 extends LinearOpMode{
    DcMotorEx MotorFL, MotorFR, MotorBL, MotorBR, ArmL, ArmR, Slide;

    Servo AxonMicro, IntakeRotate, DiffyL, DiffyR, Claw, Clean;

    CRServo RollerL, RollerR;

    AnalogInput ArmEncoder;

    private PIDController controller;

    DigitalChannel pin0;

    DigitalChannel pin1;

    private DistanceSensor frontDis;

    public double arm_pos = 90;

    public static boolean blue = true;

    public static String color_check = "B";

    public double manual_adj = 0.0;

    double power;

    ColorSensor Color;

    int argb = 0;

    boolean roller_flag = false;

    ElapsedTime timer1 = new ElapsedTime();

    ElapsedTime timer2 = new ElapsedTime();


    ElapsedTime rollertimer = new ElapsedTime();


    Boolean slidetimer = false;

    public enum State {
        IDLE,
        SAMPLEINTAKE,
        SPECINTAKE,
        SPECOUTTAKE,

    }


    // debug stuff
    double offset = -86 + 360;// -200+360

    double angle = 0;

    boolean state0;
    boolean state1;

    boolean highside = false;

    String result = "";

    public static double p = 0.0075, i = 0, d = 0.0002;

    public static double leftdiffy = 0.5, rightdiffy = 0.5, clawpos = 0.0;

    double pid;
    double ff;

    public static double f = 0.04;


//    public static double k = 0.0003;// the peak power is about 0.7 without p .
    public static int target = 180;
    public static int deadband = 1;


    //booleans

    boolean idleflag = true, misintakeflag = false;

    double claw_open = 0.45, claw_close = 0.87;

    double Ldiffy_idle = 0.1, Ldiffy_intake = 0.46, Ldiffy_outtake = 0.93;

    double Rdiffy_idle = 0.92, Rdiffy_intake = 0.56, Rdiffy_outtake = 0.5;

    double midarmspeed = 0.006, higharmspeed = 0.012;

    int arm_intake = 214, arm_outtake = 21, arm_intake_trans = 180;

    double block_open = 0.13, block_close = 0.25;



    int slide_idle = 195, slide_intake_ready = 40, slide_extend = 1800 ;

    double sweep_idle = 0.28, sweep_extend = 0.6;

    double handle_idle = 0.3,handle_trans = 0.08, handle_intake = 0.582;







    State state = State.IDLE;


    IMU IMU;

    int position=0;
    public void runOpMode(){

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        Hwinit();

        telemetry.addLine("Driver please press X for blue and O for red ");
        telemetry.update();

        while(true){
            if(gamepad1.cross){
                blue = true;
                color_check = "B";
                telemetry.addLine("Blue alliance selected!");
                break;
            }

            if (gamepad1.circle){
                blue = false;
                color_check = "R";
                telemetry.addLine("Red alliance selected!");
                break;
            }
        }

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        timer1.reset();
        timer2.reset();
        rollertimer.reset();

        initialize();

        while (opModeIsActive()){
            switch (state){

                case IDLE:
                    if (idleflag){
                        intake_idle();
                    }

                    if (gamepad2.left_bumper){
                        SpecIntakeReady();
                        state = State.SPECINTAKE;
                        break;
                    }

                    if (gamepad2.right_bumper){
                        IntakeReady();
                        state = State.SAMPLEINTAKE;
                        break;
                    }

                    if (gamepad2.right_trigger > 0.6){
                        IntakeReadyNoSweep();
                        state = State.SAMPLEINTAKE;
                        break;
                    }


                    break;
                case SAMPLEINTAKE:
                    if (gamepad1.right_bumper){


                        if (misintakeflag) IntakeReady();

                        IntakeSample();
                    }

                    if (gamepad1.right_trigger>0.6){

                        if (misintakeflag) IntakeReadyNoSweep();
                        IntakeSample();
                    }

                    if (gamepad2.left_bumper){
                        SpecIntakeReady();
                        state = State.SPECINTAKE;
                        break;
                    }

                    if (gamepad1.left_bumper){
                        intake_idle();
                        reset_arm_pos();



                        state = State.IDLE;
                        break;
                    }

                    if (gamepad2.right_bumper){
                        HPDrop();
                        state = State.SPECINTAKE;
                        break;
                    }
                    break;

                case SPECINTAKE:

                    if (roller_flag && rollertimer.milliseconds() >=750){
                        roller_flag = false;
                        RollerL.setPower(0);
                        RollerR.setPower(0);

                        SpecIntakeReady();
                    }

                    if (gamepad2.right_bumper){
                        HPDrop();
                    }

                    if (gamepad2.dpad_up &&timer2.milliseconds() > 300  && manual_adj >-0.025){


                        manual_adj-=0.015;

                        DiffyL.setPosition(Ldiffy_intake+manual_adj);
                        DiffyR.setPosition(Rdiffy_intake-manual_adj);
                        timer2.reset();

                    }

                    if (gamepad2.dpad_down && timer2.milliseconds() > 300  && manual_adj < 0.025){

                        manual_adj+=0.015;

                        DiffyL.setPosition(Ldiffy_intake+manual_adj);
                        DiffyR.setPosition(Rdiffy_intake-manual_adj);
                        timer2.reset();

                    }

                    if (gamepad1.right_bumper){

                        SpecGrab();

                        state = State.SPECOUTTAKE;
                        break;

                    }

                    if (gamepad1.left_bumper){
                        intake_idle();

                        state = State.IDLE;
                        break;
                    }
                    break;

                case SPECOUTTAKE:
                    if (gamepad2.dpad_up &&timer2.milliseconds() > 300  && manual_adj <0.025){


                        manual_adj+=0.015;

                        DiffyL.setPosition(Ldiffy_outtake+manual_adj);
                        DiffyR.setPosition(Rdiffy_outtake-manual_adj);
                        timer2.reset();

                    }

                    if (gamepad2.dpad_down && timer2.milliseconds() > 300  && manual_adj > -0.025){

                        manual_adj-=0.015;

                        DiffyL.setPosition(Ldiffy_outtake+manual_adj);
                        DiffyR.setPosition(Rdiffy_outtake-manual_adj);
                        timer2.reset();

                    }


                    if (gamepad1.right_bumper){

                        spec_outtake();

                        state = State.IDLE;
                        break;
                    }

                    if (gamepad1.left_bumper){

                        SpecIntakeReady();


                        state = State.SPECINTAKE;
                        break;

                    }
                    break;


            }







            armrotatePIDF();
//
//            argb = Color.argb();
//
////            Color.a
////
//            int red = (argb >> 16) & 0xFF;
//            int green = (argb >> 8) & 0xFF;
//            int blue = (argb) & 0xFF;




//            telemetry.addData("target", target);
//            telemetry.addData("Slide pos",Slide.getCurrentPosition());
//            telemetry.addData("Slide", Slide.getCurrentPosition());
//            telemetry.addData("Current Angle", angle);
//            telemetry.addData("Total Power", power);

            boolean state0 = pin0.getState();
            boolean state1 = pin1.getState();
            if (state0 && state1) telemetry.addLine("Yellow Detect");
            else if (state0) telemetry.addLine("BLUE Detect");
            else if (state1) telemetry.addLine("RED Detect");
            else telemetry.addLine("NA/Out of Range Detect");

            telemetry.addData("Linear slide pos",Slide.getCurrentPosition());


//            telemetry.addData("Distance sensor", frontDis.getDistance(DistanceUnit.MM));

//            telemetry.addData("Red", Color.red());
//            telemetry.addData("Blue", Color.blue());
//            telemetry.addData("Green", Color.green());
////
//            telemetry.addData("calc Red", red);
//            telemetry.addData("calc Blue", green);
//            telemetry.addData("calc Green", blue);


            telemetry.update();
            Drive(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);



        }


    }


    public void SpecIntakeReady(){
        manual_adj = 0.0;
        IntakeRotate.setPosition(handle_trans);
        DiffyL.setPosition(Ldiffy_intake);
        DiffyR.setPosition(Rdiffy_intake);
        p = midarmspeed;
        target = arm_intake;
        Claw.setPosition(claw_open);

    }

    public void SpecGrab(){
        RollerL.setPower(-1.0);
        RollerR.setPower(-1.0);
        Claw.setPosition(claw_close);
        manual_adj = 0.0;
        delay(250);
        moveStraight(0.65);
        p = higharmspeed;
        target = arm_outtake;
        delay(200);
        RollerL.setPower(0);
        RollerR.setPower(0);
        DiffyL.setPosition(Ldiffy_outtake);
        DiffyR.setPosition(Rdiffy_outtake);
        moveStraight(0);
    }



    void Hwinit() {
        controller = new PIDController(p,i,d);
        MotorFL = hardwareMap.get(DcMotorEx.class, "MotorFL");
        MotorFR = hardwareMap.get(DcMotorEx.class, "MotorFR");
        MotorBL = hardwareMap.get(DcMotorEx.class, "MotorBL");
        MotorBR = hardwareMap.get(DcMotorEx.class, "MotorBR");
        ArmL = hardwareMap.get(DcMotorEx.class, "ArmL");
        ArmR = hardwareMap.get(DcMotorEx.class, "ArmR");
        Slide = hardwareMap.get(DcMotorEx.class, "Slide");
        AxonMicro = hardwareMap.get(Servo.class, "AxonMicro");
        IntakeRotate = hardwareMap.get(Servo.class, "IntakeRotate");
        Clean = hardwareMap.get(Servo.class, "Clean");
        RollerL = hardwareMap.get(CRServo.class, "RollerL");
        RollerR = hardwareMap.get(CRServo.class, "RollerR");
        Claw = hardwareMap.get(Servo.class, "Claw");
        DiffyL = hardwareMap.get(Servo.class, "DiffyL");
        DiffyR = hardwareMap.get(Servo.class, "DiffyR");
        pin0 = hardwareMap.get(DigitalChannel.class,"pin0");
        pin1 = hardwareMap.get(DigitalChannel.class,"pin1");

        IMU    = hardwareMap.get(IMU.class,"IMU");
        ArmEncoder = hardwareMap.get(AnalogInput.class, "ArmEncoder");
        frontDis = hardwareMap.get(DistanceSensor.class, "frontDis");

        MotorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setTargetPosition(0);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setVelocity(0);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        ArmL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ArmL.setDirection(DcMotorSimple.Direction.FORWARD);
        ArmR.setDirection(DcMotorSimple.Direction.FORWARD);




        IMU.Parameters imuparameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        IMU.initialize(imuparameters);


        MotorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        Slide.setDirection(DcMotorSimple.Direction.REVERSE);
        AxonMicro.setDirection(Servo.Direction.REVERSE);


    }

    void initialize(){
        IntakeRotate.setPosition(handle_intake-0.06);
        AxonMicro.setPosition(block_open);
        Clean.setPosition(sweep_idle);
        Claw.setPosition(claw_open);
        delay(450);
        Slide.setTargetPosition(slide_idle);
        Slide.setVelocity(2800);
        delay(150);
        IntakeRotate.setPosition(handle_idle);
    }

    void Drive(double y,double x, double rx ) {

//        y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
//        x = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
//        rx = gamepad1.left_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double FLPower = (-y + x + rx) / denominator;
        double BLPower = (-y - x + rx) / denominator;
        double FRPower = (-y - x - rx) / denominator;
        double BRPower = (-y + x - rx) / denominator;
        MotorFL.setPower(FLPower);
        MotorBL.setPower(BLPower);
        MotorFR.setPower(FRPower);
        MotorBR.setPower(BRPower);
    }


    void HPDrop(){
        IntakeRotate.setPosition(handle_trans);
        RollerL.setPower(1);
        RollerR.setPower(-1);
        rollertimer.reset();
        roller_flag = true;
    }
    void IntakeReady(){

        highside = false;
        moveStraight(0.3);
        if (!misintakeflag){
            Clean.setPosition(sweep_extend);
            delay(300);
            Clean.setPosition(sweep_idle);
        }
        else{
            delay(250);
        }
        moveStraight(0);
        IntakeRotate.setPosition(handle_intake-0.04);
        AxonMicro.setPosition(block_open);
        Slide.setTargetPosition(slide_intake_ready);
        Slide.setVelocity(1500);
        RollerL.setPower(-0.3);
        RollerR.setPower(0.3);


//        sleep(200);
    }

    void IntakeReadyNoSweep(){

        highside = true;
        moveStraight(0.3);
        delay(300);
        moveStraight(0);
        IntakeRotate.setPosition(handle_intake-0.04);
        AxonMicro.setPosition(block_open);
        Slide.setTargetPosition(slide_intake_ready);
        Slide.setVelocity(1500);
        RollerL.setPower(-0.3);
        RollerR.setPower(0.3);


//        sleep(200);
    }


    void IntakeSample(){

//        total_color = Color.alpha();

        Slide.setTargetPosition(slide_extend);
        Slide.setVelocity(1000);
        IntakeRotate.setPosition(handle_intake);
        while(opModeIsActive()){

            state0 = pin0.getState();
            state1 = pin1.getState();

            if (state0 && state1) result = "Y";
            else if (state0) result = "B";
            else if (state1) result = "R";
            else result = "NA";

            Drive(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
            if (result == color_check){
                AxonMicro.setPosition(block_close);
                Slide.setVelocity(0);




                misintakeflag = false;




                delay(100);

                RollerL.setPower(0);
                RollerR.setPower(0);

                Slide.setTargetPosition(slide_idle);
                Slide.setVelocity(2800);

                if (!highside) IntakeRotate.setPosition(handle_trans);
                else IntakeRotate.setPosition(handle_idle);

                return;
            }

            if (gamepad1.left_bumper){
                misintakeflag = true;

            }

            if (Slide.getCurrentPosition()>1750 && !slidetimer){

                timer1.reset();
                slidetimer = true;
//                reset_arm_pos();
            }

            if (slidetimer && timer1.milliseconds() > 500){

                slidetimer = false;
                misintakeflag = true;
                reset_arm_pos();
                return;

            }
        }
    }

    public void intake_idle(){

        IntakeRotate.setPosition(handle_idle);

        manual_adj = 0.0;
        RollerL.setPower(0);
        RollerR.setPower(0);
        p = midarmspeed;
        slidetimer = false;
        target = arm_intake_trans;
        Claw.setPosition(claw_open);
        DiffyL.setPosition(Ldiffy_idle);
        DiffyR.setPosition(Rdiffy_idle);
        // linear slide idle pos
        // slide handle idle pos
        idleflag = false;



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


        angle = (ArmEncoder.getVoltage() / 3.2 * 360 + offset) % 360;
        if (angle <= 360 && angle >= 250) angle -=360;

        return angle;

    }

    public void reset_arm_pos() {
        Clean.setPosition(sweep_idle);
        Slide.setTargetPosition(slide_idle);
        Slide.setVelocity(2800);
        IntakeRotate.setPosition(handle_idle);
        AxonMicro.setPosition(block_open);
        delay(200);
        RollerL.setPower(0);
        RollerR.setPower(0);

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
        ArmL.setPower(-power);
        ArmR.setPower(power);


    }

    private double square_root(double input) {
        if (input >= 0) {
            return Math.sqrt(input);
        } else {
            return -1 * Math.sqrt(Math.abs(input));

        }
    }

    public void moveStraight (double pow){
        MotorFL.setPower(pow);
        MotorBL.setPower(pow);
        MotorFR.setPower(pow);
        MotorBR.setPower(pow);
    }

    public void spec_outtake(){

        IntakeRotate.setPosition(handle_idle);

        idleflag = true;
        manual_adj = 0.0;
        moveStraight(0.7);
        delay(150);
        while (opModeIsActive() && frontDis.getDistance(DistanceUnit.MM) > 120){

        }
        moveStraight(0);
        delay(50);
        Claw.setPosition(claw_open);

        delay(50);
        moveStraight(-1.0);
        delay(250);

    }
}
