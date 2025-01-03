package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "Debug", group = "B")
public class Debug extends LinearOpMode {
    BaseClass rbg;


//regional

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.addLine("Wait ! Initializing............. ");
        telemetry.update();
        Pose2d p1 = new Pose2d(0, 0, 0);

        rbg = new BaseClass(this, p1);
        double tmp=0;
        int tar=0;
//        rbg.Gearbox.setPosition(0);
        rbg.Intake_rot.setPosition(0);
        rbg.Arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rbg.Arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbg.Arm_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rbg.Arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rbg.Arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbg.Arm_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rbg.Slide_bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbg.Slide_bot.setTargetPosition(0);
        rbg.Slide_bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbg.Slide_bot.setVelocity(0);

        rbg.Slide_top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbg.Slide_top.setTargetPosition(0);
        rbg.Slide_top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbg.Slide_top.setVelocity(0);


        if (isStopRequested()) return;

        telemetry.addLine("Press Start Now!:");
        telemetry.update();
        rbg.timer(0,0);
        rbg.init(0);

        rbg.Gearbox.setPosition(0.95); // 0.05
//        rbg.init(2);

        //    rbg.rotatetargetPIDF(rbg.rotateStart);
        waitForStart();

        rbg.timer(0, rbg.start);

        rbg.Arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbg.Arm_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rbg.Arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbg.Arm_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // rbg.Intake_rot.setPosition(0.3);
        //rbg.Intake_rot.setPosition(0.3);
        sleep(500);

        rbg.Slide_bot.setTargetPosition(0);
       rbg.Slide_bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbg.Slide_bot.setVelocity(0);


        rbg.Slide_top.setTargetPosition(0);
        rbg.Slide_top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbg.Slide_top.setVelocity(0);
        rbg.Arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbg.Arm_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rbg.Arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbg.Arm_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // rbg.Intake_rot.setPosition(0.3);
        //rbg.Intake_rot.setPosition(0.3);
        sleep(500);


        while (opModeIsActive()) {
           // rbg.Intake_rot.setPosition(0);


        if(rbg.timer(250,0)) {
//            if (gamepad2.triangle) {rbg.armslidePos+= rbg.armslideStep; rbg.timer(0,0);}
//            if (gamepad2.cross) {rbg.armslidePos -= rbg.armslideStep;; rbg.timer(0,0);}
//            if (gamepad2.square) {rbg.armrotatePos -=rbg.armrotateStep; rbg.timer(0,0);}
//            if (gamepad2.square) {rbg.armrotatePos-= ; rbg.timer(0,0);}
//            if (gamepad2.circle) {rbg.armrotatePos += rbg.armrotateStep;; rbg.timer(0,0);}
            if (gamepad2.dpad_up) {
                rbg.handlePos += rbg.handleStep;;

                rbg.Intake_handle.setPosition(rbg.handlePos);
                rbg.timer(0, 0);
            }
            if (gamepad2.dpad_down) {
                rbg.handlePos -= rbg.handleStep;
                rbg.Intake_handle.setPosition(rbg.handlePos);
                rbg.timer(0, 0);
            }
            if (gamepad2.dpad_left) {
                rbg.intakerotpose -= rbg.clawStep;
                rbg.Intake_rot.setPosition(rbg.intakerotpose);
                rbg.timer(0, 0);
            }
            if (gamepad2.dpad_right) {
                rbg.intakerotpose += rbg.clawStep;
                rbg.Intake_rot.setPosition(rbg.intakerotpose);
                rbg.timer(0, 0);
            }
            if (gamepad2.triangle) {
                rbg.gearboxpose += rbg.clawStep;
                rbg.Gearbox.setPosition(rbg.gearboxpose);
                rbg.timer(0, 0);
            }
            if (gamepad2.cross) {
                rbg.gearboxpose -= rbg.clawStep;
                rbg.Gearbox.setPosition(rbg.gearboxpose);
                rbg.timer(0, 0);
            }



            if (gamepad2.circle) {
                rbg.Intake.setPower(0.8);
                sleep(500);
                rbg.Intake.setPower(0);
            }
            if (gamepad2.square) {
                rbg.Intake.setPower(-0.8);
                sleep(500);
                rbg.Intake.setPower(0);
            }
            if(gamepad1.dpad_up)
            {
                tar=tar+100;
                rbg.linearslide(tar,2700);



            }
            if(gamepad1.dpad_down)
            {
                tar=tar-100;
                rbg.linearslide(tar,2700);

            }

//            if (gamepad1.left_bumper) {
//               // rbg.preintake();
//            }
//
//            if (gamepad1.right_bumper) {
//                rbg.intake();
//            }

            if (gamepad1.right_trigger > 0.4){
                rbg.Slide_top.setPower(0.3);
                sleep(500);
                rbg.Slide_top.setPower(0);

            }
            if (gamepad1.left_trigger > 0.4){
                rbg.Slide_bot.setPower(0.3);
                sleep(500);
                rbg.Slide_bot.setPower(0);

            }

            if (gamepad1.left_bumper){
                rbg.outtake_spec_pre();
            }

            if (gamepad1.right_bumper){
                rbg.outtake_spec_test();

            }

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
        }

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



     rbg.armrotatePIDF();
         //   rbg.armrotatePIDF();
          //  rbg.armrotatePIDF();
            telemetry.addData("armlinerslide", rbg.Slide_top.getCurrentPosition());
            telemetry.addData("armrotate position", -rbg.Arm_right.getCurrentPosition());


//            telemetry.addData("Colors red", rbg.Intake_color.red());
//            telemetry.addData("Colors green", rbg.Intake_color.green());
//            telemetry.addData("Colors blue", rbg.Intake_color.blue());


            telemetry.addData("Intake HANDLE", rbg.  Intake_handle.getPosition());
            telemetry.addData("Intake rot", rbg.Intake_rot.getPosition());
          //  telemetry.addData("Intake rot", rbg.Intake_rot.getPosition());
           telemetry.addData("gerbox", rbg.Gearbox.getPosition());
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
            telemetry.addData("Bar distance", rbg.bar_dist.getDistance(DistanceUnit.MM));
            telemetry.addData("Basket distance", rbg.basket_dist.getDistance(DistanceUnit.MM));
      //      telemetry.addLine("gamepad2 right/lefit bumper   handle rot");

            telemetry.addData("Intake rot",rbg.intakerotpose);
            telemetry.update();
          //  update_flag=false;








            }
        }
    }










