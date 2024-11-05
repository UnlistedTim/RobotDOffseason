//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//
//import org.firstinspires.ftc.robotcore.internal.system.Deadline;
//
//import java.util.concurrent.TimeUnit;
//
//@Autonomous(name = "Automeet1", group = "A")
////@Disabled
//@Config
//@Disabled
//public class Automeet extends LinearOpMode {
//    // private ElapsedTime runtime = new ElapsedTime();
//
//    boolean setup_ready = false;
//    // boolean debuging_mode=false;// Need turn off for game
//    long delay = 0;
//    Pose2d pp=new Pose2d(0, 0, 0);;
//
//    BaseClass rbga;
//    Action tr1;
//    Action tr2;
//    Action test;
//
//
//    Action tb1;
//   Action tb2;
////    Action tb3;
////
////    Action pre2;
////    Action pre1;
////    Action pre3;
////    Action bpre1;
////    Action bpre2;
////    Action bpre3;
////    Action bturn1back;
////    Action turn1back;
////    int path=3;
//
//
//
//
//
//    Pose2d start=new Pose2d(0,0,Math.toRadians(0));
//    Pose2d r1= new Pose2d(16,32,Math.toRadians(-45));
//    Pose2d r2= new Pose2d(22,-12,Math.toRadians(15));
//    double x,y, heading;
//    Vector2d r3= new Vector2d(31,-49);
//    //Pose2d r3= new Pose2d(2.5,0,Math.toRadians(-9)); // -10.5
//
//    Pose2d rpreout1 = new Pose2d(89, -50, Math.toRadians(84)); // 91
//    Pose2d rpreout2 = new Pose2d(68, -50, Math.toRadians(82));
//    Pose2d rpreout3 = new Pose2d(53, -50,Math.toRadians(82));
//
//
//
//
//
//    Pose2d b1= new Pose2d(15,34,Math.toRadians(-45));
//    Pose2d b2= new Pose2d(14,-60,Math.toRadians(45));
//    Pose2d b3= new Pose2d(2.5,0,Math.toRadians(-10.5));
//
//
//    Pose2d bpreout1 = new Pose2d(49,50,Math.toRadians(-84));
//    Pose2d bpreout2 = new Pose2d(30,50,Math.toRadians(-83));  //66
//    Pose2d bpreout3 = new Pose2d(60, 50, Math.toRadians(-82)); //80
//
//
//
//    Pose2d t1back = new Pose2d(2.5,0,Math.toRadians(0));
//    Pose2d bt1back = new Pose2d(2.5,0,Math.toRadians(0));
//
//    @Override
//    public void runOpMode() {
//        rbga = new BaseClass(this, pp);
//        rbga.baseblue = false;
//        rbga.baseright = false;
////        tr1 = rbga.actionBuilder(start)
////                .lineToX(2.5)
////                .turn(Math.toRadians(11))
//////                .waitSeconds(2)
//////                .setTangent(Math.toRadians(90))
//////                .lineToY(48)
//////                .setTangent(Math.toRadians(0))
//////                .lineToX(32)
//////                .strafeTo(new Vector2d(44.5, 30))
//////                .turn(Math.toRadians(180))
//////                .lineToX(47.5)
//////                .waitSeconds(3)
////                .build();
//
//        tr1 = rbga.actionBuilder(start)
//                .splineToSplineHeading(r1,0)
//               // .turn(Math.toRadians(11))
////                .waitSeconds(2)
////                .setTangent(Math.toRadians(90))
////                .lineToY(48)
////                .setTangent(Math.toRadians(0))
////                .lineToX(32)
////                .strafeTo(new Vector2d(44.5, 30))
////                .turn(Math.toRadians(180))
////                .lineToX(47.5)
////                .waitSeconds(3)
//                .build();
//
//
//        tr2 = rbga.actionBuilder(r1)
//                .splineToLinearHeading(r2,0)
//                .splineToConstantHeading(r3,0)
//                // .turn(Math.toRadians(11))
////                .waitSeconds(2)
////                .setTangent(Math.toRadians(90))
////                .lineToY(48)
////                .setTangent(Math.toRadians(0))
////                .lineToX(32)
////                .strafeTo(new Vector2d(44.5, 30))
////                .turn(Math.toRadians(180))
////                .lineToX(47.5)
////                .waitSeconds(3)
//                .build();
//
//
//
////        turn1back = rbga.actionBuilder(r1)
////                .turn(Math.toRadians(-11))
////                .build();
////
////        bturn1back = rbga.actionBuilder(b3)
////                .turn(Math.toRadians(25))// 10.5 // 10.5
////                .build();
////        tr2 = rbga.actionBuilder(start)
//////                .splineToLinearHeading(r2, 0)
//////                .waitSeconds(2)
//////                .setTangent(Math.toRadians(90))
////                .lineToX(4.5)
//////                .setTangent(Math.toRadians(0))
//////                .lineToX(32)
//////                .strafeTo(new Vector2d(44.5, 30))
//////                .turn(Math.toRadians(180))
//////                .lineToX(47.5)
//////                .waitSeconds(3)
////                .build();
//
////        tb2 = rbga.actionBuilder(start)
////                .lineToX(4.5)
//////                .waitSeconds(2)
//////                .setTangent(Math.toRadians(90))
//////                .lineToY(48)
//////                .setTangent(Math.toRadians(0))
//////                .lineToX(32)
//////                .strafeTo(new Vector2d(44.5, 30))
//////                .turn(Math.toRadians(180))
//////                .lineToX(47.5)
//////                .waitSeconds(3)
////                .build();
////
////        tr3 = rbga.actionBuilder(start)
////                .lineToX(2.5)
////                .turn(Math.toRadians(-9))
//////                .waitSeconds(2)
//////                .setTangent(Math.toRadians(90))
//////                .setTangent(Math.toRadians(0))
//////                .lineToX(32)
//////                .strafeTo(new Vector2d(44.5, 30))
//////                .turn(Math.toRadians(180))
//////                .lineToX(47.5)
//////                .waitSeconds(3)
////                .build();
//
////        tb3 = rbga.actionBuilder(start)
////                .lineToX(2.5)
////                .turn(Math.toRadians(-10.5))
//////                .waitSeconds(2)
//////                .setTangent(Math.toRadians(90))
//////                .setTangent(Math.toRadians(0))
//////                .lineToX(32)
//////                .strafeTo(new Vector2d(44.5, 30))
//////                .turn(Math.toRadians(180))
//////                .lineToX(47.5)
//////                .waitSeconds(3)
////                .build();
//
//
////
////        pre1 = rbga.actionBuilder(r1)
////                .splineToLinearHeading(rpreout1,0)
////                .build();
////        pre2 = rbga.actionBuilder(r2)
////                .splineToLinearHeading(rpreout2,0)
////                .build();
////        pre3 = rbga.actionBuilder(r3)
////                .splineToLinearHeading(rpreout3,0)
////                .build();
//
//
//
//
//
//
////
////        bpre1 = rbga.actionBuilder(b1)
////                .splineToLinearHeading(bpreout1,0)
////                .build();
////
////        bpre2 = rbga.actionBuilder(b2)
////                .splineToSplineHeading(bpreout2,0)
////                .build();
////
////
////        bpre3 = rbga.actionBuilder(b3)
////                .splineToLinearHeading(bpreout3,0)
////                .build();
//        rbga.Claw.setPosition(rbga.clawClose);//
//        rbga.Handle.setPosition(0);
//        rbga.linearslides_reset();
//        sleep(2000);
//        telemetry.addData(">", "Press Play to start op mode ");
//        x =rbga.pose.position.x;
//        y=rbga.pose.position.y;
//        heading=Math.toDegrees(rbga.pose.heading.toDouble());
//        telemetry.update();
//
//        //  configuration_info();
//        waitForStart();
//      //  rbga.endgame = rbga.runtime.seconds() + 30;
//
//
//
//        while (opModeIsActive()) {
//
//
//
//            Actions.runBlocking(
//                    new SequentialAction(
//                            tr1
//                    )
//
//            );
//            rbga.aouttake();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            tr2
//                    )
//
//            );
//            rbga.stop_drive();
//
//            rbga. move(-0.3);
//            sleep(1200); //800
//            rbga.stop_drive();
//            sleep(500);
//            return;
//
//        }
//    }
//
//
//    void configuration_info() {
//        telemetry.addLine("0.Gunner-- right bumper :Load the Sample   ) ");
//        telemetry.update();
//
//
//
//        while (!setup_ready && !isStopRequested()) {
//
////        if (gamepad2.dpad_up || gamepad2.dpad_down) {
////
////            if(gamepad2.dpad_up ) delay=delay+5000;
////            if(gamepad2.dpad_down ) delay=delay-5000;
////            if (delay>15000) delay=15000;
////            if (delay<0)  delay=0;
////            telemetry.addData("Delay Time",delay);
////            telemetry.update();
////        }
//
//            if(gamepad2.right_bumper)
//            {
//                rbga.Claw.setPosition(rbga.clawClose);//
//                rbga.Handle.setPosition(0);
//            }
//            ;
//
////            if (gamepad1.dpad_up || gamepad1.dpad_down) {
////
////                sleep(300);
////            }
////            if (gamepad1.dpad_right || gamepad1.dpad_left) {
////
////                sleep(300);
////            }
//
////            if (gamepad1.b || gamepad1.circle) {
////                rbga.baseright = true;
////            }
////            if (gamepad1.x || gamepad1.square) {
////
////                rbga.baseright = false;
////
////            }
//
//            if (gamepad2.a || gamepad2.cross) {
//
//                rbga.baseblue = true;
//
//            }
//            if (gamepad2.b || gamepad2.circle) {
//
//                rbga.baseblue = false;
//
//            }
//
////        if (gamepad2.y || gamepad2.triangle) {
////
////            rbga.lgrab();
////
////        }
//
//
//
//
////            if (rbga.baseright)
////                telemetry.addLine(" Right   SIDE ");
////            else
////                telemetry.addLine(" Left   SIDE ");
////            telemetry.addLine();
//
//            if (rbga.baseblue)
//                telemetry.addLine(" Blue   Color ");
//            else
//                telemetry.addLine(" Red   Color ");
//
//            telemetry.addLine();
//            telemetry.addLine();
//            telemetry.addLine();
//            telemetry.addLine("0. Gunner: Right bumper- Load  the Sample ");
//      //      telemetry.addLine("1. Load  yellow and purple pixels ");
//         //   telemetry.addLine("2. Driver:  LEFT:Square  RIGHT:Circle ");
//            telemetry.addLine("3. Gunner:   BLUE:Cross(A)    RED:Circle(B)");
//            telemetry.addLine("4. Place the Robot with right location  ");
//        //    telemetry.addLine("6. Optional: Add delay time.  Gunner: Dpad-Up +5s(max 15s), Dapd-Down -5s  ");
//            telemetry.addLine("7.  Coach double check and press Driver-- Triangle key ");
//          //  telemetry.addLine("8.  Check the huskeylens screen to make sure the ID is correct");
//            telemetry.update();
//
//
//            if (gamepad1.triangle) setup_ready = true;
//
//        }
////
//       // rbga.linearslides_reset();
//       // rbga.lazyImu.
////
////
////    if (rbga.baseblue) {
////        telemetry.addLine("BLUE Color confirmed ");
//////       rbga.base_align_angle = -90;
//////        rbga.base_apr_id = 1;
////    }
////    else {
////        telemetry.addLine("RED Color confirmed ");
//////        rbga.base_align_angle = 90;
//////        rbga.base_apr_id = 4;
////
////    }
////
////    telemetry.addLine();
//
//
////    if (rbga.baseright)
////        telemetry.addLine("RIGHT Side confirmed ");
////    else
////        telemetry.addLine("LEFT Side confirmed ");
//
////    if (rbga.outtake_1_delay > 0){
////        telemetry.addData("First Outtake Delay @ Backstage Wall side (ms)",rbga.outtake_1_delay);
////    }
////    if (rbga.outtake_2_delay > 0){
//////        telemetry.addData("Second Outtake Delay @ Backstage Wall side (ms)",rbga.outtake_2_delay);
////    }
//
////    telemetry.addLine("Check the huskeylens ID. If not correct, restart the program ");
////
////    telemetry.addLine();
//
//  //  if(delay>0)   telemetry.addData("Delay Time(seconds)",delay);
//  //  telemetry.update();
// //   telemetry.addLine();
//    /** Wait for the game to begin */
//  //  telemetry.addData("Status", "Initialized");
//    telemetry.addData(">", "Press Play to start op mode ");
//    telemetry.update();
//
//
//
//    }
////    void bautoroutepre3(){
////        Actions.runBlocking(
////                new SequentialAction(
////                        bpre3
////                )
////        );
////
////        rbga.stop_drive();
////
////    }
//}
//
////
