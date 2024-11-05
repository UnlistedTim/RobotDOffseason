package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "Debug", group = "B")
public class Debug extends LinearOpMode {
    BaseClass rbg;




    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.addLine("Wait ! Initializing............. ");
        telemetry.update();
        Pose2d p1 = new Pose2d(0, 0, 0);

        rbg = new BaseClass(this, p1);

        if (isStopRequested()) return;

        telemetry.addLine("Press Start Now!:");
        telemetry.update();
        rbg.timer(0,0);
        rbg.init(0);
//        rbg.init(2);

        //    rbg.rotatetargetPIDF(rbg.rotateStart);
        waitForStart();

        rbg.Intake_rot.setPosition(0);


        while (opModeIsActive()) {
           // rbg.Intake_rot.setPosition(0);


        if(rbg.timer(250,0)) {
//            if (gamepad2.triangle) {rbg.armslidePos+= rbg.armslideStep; rbg.timer(0,0);}
//            if (gamepad2.cross) {rbg.armslidePos -= rbg.armslideStep;; rbg.timer(0,0);}
//         //   if (gamepad2.square) {rbg.armrotatePos -=rbg.armrotateStep; rbg.timer(0,0);}
//           // if (gamepad2.square) {rbg.armrotatePos-= ; rbg.timer(0,0);}
//         //   if (gamepad2.circle) {rbg.armrotatePos += rbg.armrotateStep;; rbg.timer(0,0);}
//            if (gamepad2.dpad_up) {rbg.handlePos -= rbg.handleStep;; rbg.timer(0,0);}
//            if (gamepad2.dpad_down) {rbg.handlePos += rbg.handleStep;; rbg.timer(0,0);}
//            if (gamepad2.dpad_left) {rbg.clawPos -= rbg.clawStep;; rbg.timer(0,0);}
//            if (gamepad2.dpad_right) {rbg.clawPos += rbg.clawStep;; rbg.timer(0,0);}
////
        }
//
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
//            {
//                if(rbg.handlePos<0) rbg.handlePos=0;
//                if(rbg.handlePos>1 ) rbg.handlePos=1;
//                rbg.Handle.setPosition(rbg.handlePos);
//                rbg.handleCurrent=rbg.handlePos;
//                update_flag=true;
//            }
//
//
//        if(Math.abs(rbg.clawPos-rbg.clawCurrent)>0.001)
//        {
//            if(rbg.clawPos<0) rbg.clawPos=0;
//            if(rbg.clawPos>1 ) rbg.clawPos=1;
//           // rbg.armSlide.getCurrentAlert(CurrentUnit.MILLIAMPS);
//
//            rbg.Claw.setPosition(rbg.clawPos);
//            rbg.clawCurrent=rbg.clawPos;
//            update_flag=true;
//        }
//        rbg.rotatetargetPIDF(rbg.armrotatePos)rbg.rotatetargetPIDF(rbg.armrotatePos)
//      //  rbg.rotatetargetPIDF(rbg.armrotatePos);
//     //   rbg.armrotatePIDF();
//     //   rbg.robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, 0.5);
     if(gamepad1.left_bumper) {

         rbg.Intake_handle.setPosition(rbg.handleOutSpeca);
         rbg.rotatetargetPIDF(rbg.rotate[rbg.souta]);
         rbg.armrotatePIDF();
         rbg.delay(2000);


     }

     if(gamepad1.right_bumper) {
                rbg.stop_drive();
                rbg.move(-0.16);
//                while (rbg.reardis.getDistance(DistanceUnit.MM) >target && opModeIsActive()) {
//                   rbg.armrotatePIDF();
//                  // sleep(5);
//
//                }
                rbg.stop_drive();
            }
   //  rbg.armrotatePIDF();
         //   rbg.armrotatePIDF();
        if(true) {
            telemetry.addLine("gamepad2  x/triangle  linerside,  square/circle armrotate");
            telemetry.addLine("gamepad2  dapad down/up handle left/right claw");
        telemetry.addData("ClawPos", rbg.clawPos);
         telemetry.addData("handlePos", rbg.handlePos);
         telemetry.addData("Intake rot", rbg.Intake_rot.getPosition());
//            telemetry.addData("armlinerslide", rbg.armslidePos);
           telemetry.addData("armlinerslideread", rbg.Slide_top.getCurrentPosition());

 //           telemetry.addData("armrotate degree", rbg.armrotatePos);

           // telemetry.addData("armrotate pos ",  rbg.rotateticks);
            telemetry.addData("armrotate target", rbg.rotateTarget);
            telemetry.addData("armrotate read", rbg.Arm_left.getCurrentPosition());
        //    telemetry.addData("armrotate read", rbg.Arm_left.getCurrent(CurrentUnit.MILLIAMPS));

//            telemetry.addData("armrslide currentalert",  rbg.armSlide.getCurrentAlert(CurrentUnit.MILLIAMPS));
//            telemetry.addData("armrotate currentalert",   rbg.armRotate.getCurrentAlert(CurrentUnit.MILLIAMPS));
    //        telemetry.addData("armrslide current",  rbg.armSlide.getCurrent(CurrentUnit.MILLIAMPS));
           // telemetry.addData("armrotate current",   rbg.armRotate.getCurrent(CurrentUnit.MILLIAMPS));
   //         telemetry.addData("armrslide current",  rbg.armSlide.isOverCurrent());
       //     telemetry.addData("armrotate current",   rbg.armRotate.isOverCurrent());

            telemetry.update();
          //  update_flag=false;


        }





            }
        }
    }










