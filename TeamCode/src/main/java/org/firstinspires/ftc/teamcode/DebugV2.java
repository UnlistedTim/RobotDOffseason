package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


//CLAW POS Fully open 0.72, Fully closed 0.93;


@TeleOp(name = "DebugV2", group = "B")
public class DebugV2 extends LinearOpMode {

    double lefthandlepos = 0;
    double righthandlepos = 1.0;

    double clawpos = 0.7;

    double offset = -200 +360;


    public DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public ColorSensor Intake_color;

    double angle;



    public  DcMotorEx Arm_right, Arm_left, Slide_top,Slide_bot;
    public Servo Left_handle,Right_handle, Gearbox, Claw;
    public DigitalChannel Arm_touch;
    public VoltageSensor voltageSensor;

    public DistanceSensor bar_dist;
    public DistanceSensor basket_dist;
    public AnalogInput Arm_encoder;


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


        Arm_right = hardwareMap.get(DcMotorEx.class, "Arm_right");
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

            angle = 360 - ((Arm_encoder.getVoltage() / 3.2 * 360 + offset) % 360);
            if (angle < 360 && angle > 300) angle-=360;
           // Intake_rot.setPosition(0);


//            if (gamepad2.triangle) {armslidePos+= armslideStep; rbg.timer(0,0);}
//            if (gamepad2.cross) {rbg.armslidePos -= rbg.armslideStep;; rbg.timer(0,0);}
//            if (gamepad2.square) {rbg.armrotatePos -=rbg.armrotateStep; rbg.timer(0,0);}
//            if (gamepad2.square) {rbg.armrotatePos-= ; rbg.timer(0,0);}
//            if (gamepad2.circle) {rbg.armrotatePos += rbg.armrotateStep;; rbg.timer(0,0);}
            if (gamepad2.dpad_up) {
                if (lefthandlepos <= 0.98 && righthandlepos >=0.02){
                    lefthandlepos+=0.02;
                    righthandlepos-=0.02;
                    Left_handle.setPosition(lefthandlepos);
                    Right_handle.setPosition(righthandlepos);
                    sleep(300);
                }



            }
            if (gamepad2.dpad_down) {
                if (lefthandlepos >= 0.02 && righthandlepos <=0.98) {
                    lefthandlepos-=0.02;
                    righthandlepos+=0.02;
                    Left_handle.setPosition(lefthandlepos);
                    Right_handle.setPosition(righthandlepos);
                    sleep(300);
                }


            }
            if (gamepad2.dpad_left) {
                if (righthandlepos >= 0.02 && lefthandlepos >=0.02){
                    lefthandlepos-=0.02;
                    righthandlepos-=0.02;
                    Left_handle.setPosition(lefthandlepos);
                    Right_handle.setPosition(righthandlepos);

                    sleep(300);
                }

            }
            if (gamepad2.dpad_right) {
                if (righthandlepos <= 0.98 && lefthandlepos <=0.98){
                    lefthandlepos+=0.02;
                    righthandlepos+=0.02;
                    Left_handle.setPosition(lefthandlepos);
                    Right_handle.setPosition(righthandlepos);
                    sleep(300);
                }

            }
            if (gamepad2.triangle) {
            }
            if (gamepad2.cross) {
            }



            if (gamepad2.circle) {
            }
            if (gamepad2.square) {
            }
            if(gamepad1.dpad_up) {
                if (clawpos <= 0.99){
                    clawpos+=0.02;
                    Claw.setPosition(clawpos);
                    sleep(300);
                }
//                tar=tar+100;
//                linearslide(tar,2700);

            }
            if(gamepad1.dpad_down)
            {
                if (clawpos >= 0.01){
                    clawpos-=0.02;
                    Claw.setPosition(clawpos);
                    sleep(300);
                }

//                tar=tar-100;
//                rbg.linearslide(tar,2700);

            }

//            if (gamepad1.left_bumper) {
//               // rbg.preintake();
//            }
//
//            if (gamepad1.right_bumper) {
//                rbg.intake();
//            }



            //test

//            if (rbg.arm_rot_power <= 1.0 &&  gamepad2.right_bumper) {
//                rbg.arm_rot_power += 0.02;
//                rbg.Arm_right.setPower(-rbg.arm_rot_power);
//                rbg.Arm_left.setPower(-rbg.arm_rot_power);
//                rbg.timer(0, 0);
//            }
//
//            if (rbg.arm_rot_power >= -1.0 && gamepad2.left_bumper ) {
//
//                rbg.arm_rot_power -= 0.02;
//                rbg.Arm_right.setPower(rbg.arm_rot_power);
//                rbg.Arm_left.setPower(rbg.arm_rot_power);
//                rbg.timer(0, 0);
//            }
            
//            if (gamepad1.dpad_up){
//                rbg.arm_slide_pos +=100;
//                rbg.Slide_top.setTargetPosition(rbg.arm_slide_pos);
//                rbg.Slide_top.setVelocity(1000);
//                rbg.Slide_bot.setTargetPosition(rbg.arm_slide_pos);
//                rbg.Slide_bot.setVelocity(1000);
//                rbg.timer(0, 0);
//
//            }
//            if (gamepad1.dpad_down){
//                rbg.arm_slide_pos -=100;
//                rbg.Slide_top.setTargetPosition(rbg.arm_slide_pos);
//                rbg.Slide_top.setVelocity(1000);
//                rbg.Slide_bot.setTargetPosition(rbg.arm_slide_pos);
//                rbg.Slide_bot.setVelocity(1000);
//                rbg.timer(0, 0);
//
//            }
//            if (gamepad2.square) {rbg.armrotatePos -=rbg.armrotateStep; rbg.timer(0,0);}
//            if (gamepad2.square) {rbg.armrotatePos-= ; rbg.timer(0,0);}
//            if (gamepad2.circle) {rbg.armrotatePos += rbg.armrotateStep;; rbg.timer(0,0);}

//
//
////        if(Math.abs(rbg.armslidePos-rbg.armslideCurrent)>5){
////            if(rbg.armslidePos>rbg.armslideH) rbg.armslidePos=rbg.armslideH;
////            if(rbg.armslidePos<rbg.armslideL) rbg.armslidePos=rbg.armslideL;
////            rbg.linearslide(rbg.armSlide,rbg.armslidePos,rbg.armslideV3);
////            rbg.armslideCurrent=rbg.armslidePos;
////            update_flag=true;
////
////        }
//
////        if(Math.abs(rbg.armrotatePos-rbg.armrotateCurrent)>3){
////                if(rbg.armrotatePos>rbg.armrotateH) rbg.armrotatePos=rbg.armrotateH;
////                if(rbg.armrotatePos<rbg.armrotateL) rbg.armrotatePos=rbg.armrotateL;
////               // rbg.linearslide(rbg.armRotate,rbg.armrotatePos,rbg.armrotateV2);
////                rbg.rotatetargetPIDF(rbg.armrotatePos);
////              //  rbg.armrotatePIDF();
////                rbg.armrotateCurrent=rbg.armrotatePos;
////                update_flag=true;
////            }
//
////
//        if(Math.abs(rbg.handlePos-rbg.handleCurrent)>0.001)
       //        rbg.rotatetargetPIDF(rbg.armrotatePos)rbg.rotatetargetPIDF(rbg.armrotatePos)
//      //  rbg.rotatetargetPIDF(rbg.armrotatePos);
//     //   rbg.armrotatePIDF();
//     //   rbg.robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, 0.5);



         //   rbg.armrotatePIDF();
          //  rbg.armrotatePIDF();
            telemetry.addData("armlinerslide", Slide_top.getCurrentPosition());
            telemetry.addData("armrotate position", angle);

            telemetry.addData("left handle pos", lefthandlepos);
            telemetry.addData("right handle pos", righthandlepos);

            telemetry.addData("CLaw pos", clawpos);



//            telemetry.addData("Colors red", rbg.Intake_color.red());
//            telemetry.addData("Colors green", rbg.Intake_color.green());
//            telemetry.addData("Colors blue", rbg.Intake_color.blue());


//            telemetry.addData("Intake HANDLE", Intake_handle.getPosition());
//            telemetry.addData("Intake rot", Intake_rot.getPosition());
          //  telemetry.addData("Intake rot", rbg.Intake_rot.getPosition());
           telemetry.addData("gerbox", Gearbox.getPosition());
         //   telemetry.addData("intake handle", rbg.Intake_handle.getPosition());



           // telemetry.addData("armrotate pos ",  rbg.rotateticks);
//            telemetry.addData("armrotate target", rbg.rotateTarget);
//            telemetry.addData("armrotate read", rbg.Arm_left.getCurrentPosition());
        //    telemetry.addData("armrotate read", rbg.Arm_left.getCurrent(CurrentUnit.MILLIAMPS));

//            telemetry.addData("armrslide currentalert",  rbg.armSlide.getCurrentAlert(CurrentUnit.MILLIAMPS));
//            telemetry.addData("armrotate currentalert",   rbg.armRotate.
//
//
//            (CurrentUnit.MILLIAMPS));
    //        telemetry.addData("armrslide current",  rbg.armSlide.getCurrent(CurrentUnit.MILLIAMPS));
           // telemetry.addData("armrotate current",   rbg.armRotate.getCurrent(CurrentUnit.MILLIAMPS));
   //         telemetry.addData("armrslide current",  rbg.armSlide.isOverCurrent());
       //     telemetry.addData("armrotate current",   rbg.armRotate.isOverCurrent());

         //   telemetry.addLine("gamepad2  x/triangle  linerside,  square/circle armrotate");
            telemetry.addLine("gamepad2  dapad down/up handle        dpad left/right claw ort");
//            telemetry.addData("Bar distance", bar_dist.getDistance(DistanceUnit.MM));
//            telemetry.addData("Basket distance", rbg.basket_dist.getDistance(DistanceUnit.MM));
      //      telemetry.addLine("gamepad2 right/lefit bumper   handle rot");

//            telemetry.addData("Intake rot",intakerotpose);
            telemetry.update();
          //  update_flag=false;








            }
        }

}








