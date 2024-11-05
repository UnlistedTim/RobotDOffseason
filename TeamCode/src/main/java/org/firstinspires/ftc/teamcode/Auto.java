//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//@Autonomous(name = "Automeet2", group = "A")
////@Disabled
//@Config
//public class Auto extends LinearOpMode {
//    // private ElapsedTime runtime = new ElapsedTime();
//
//
//    Pose2d pp = new Pose2d(0, 0, 0);
//    BaseClass rbg;
//    Pose2d start = new Pose2d(0, 0, Math.toRadians(0));
//    double x, y, heading;
//
//    @Override
//    public void runOpMode() {
//        rbg = new BaseClass(this, pp);
//        sleep(200);
//        rbg.init(0);
//        rbg.init(2);
//        sleep(200);
//        telemetry.addData(">", "Press Play to start op mode ");
////        x = rbg.pose.position.x;
////        y = rbg.pose.position.y;
////        heading = Math.toDegrees(rbg.pose.heading.toDouble());
//        telemetry.update();
//
//
//        waitForStart();
//        while (opModeIsActive()) {
//
////            telemetry.addData("x", rbg.pose.position.x);
////            telemetry.addData("y", rbg.pose.position.y);
////            telemetry.addData("heading", rbg.imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES)));
////            telemetry.update();
//         //   sleep(10000);
//
//
//
////            rbg.stop_drive();
////            sleep(1000000000);
//
//
//
//
//
//
//
//            rbg.pmove(0,false); //forward
//            rbg.flag[rbg.asin]=true;
//            rbg.aspecimen_outtake();
//            rbg.flag[rbg.asin]=false;
//            rbg.pmove(1,true);//strafe for intake
//            rbg.aspicmenintake();
//             rbg.pmove(8,true); // strafe for sample outtake
//            rbg.aspecimen_outtake();
//            rbg.pmove(9,true); // strafe to push 1st sample
//            rbg.pmove(2,false);//forward to get behind 1st sample
//            rbg.pmove(3,true);//strafe to align with first sample
//            rbg.pmove(4,false);//backward to push 1st sample to hp
//            rbg.pmove(5,false);//forward go to 2nd sample
//            rbg.pmove(6,true);//strafe to align with 2nd sample
//            rbg.pmove(7,false);//backward to push 2nd sample to hp
//            rbg.flag[rbg.push]=true;
//            rbg.aspicmenintake();
//            rbg.pmove(10,true);//strafe
//            rbg.flag[rbg.asin]=true;
//            rbg.aspecimen_outtake();
//            rbg.pmove(13,true);
//
//
//
//            rbg.pmove(11,true);//strafe
//            rbg.aspicmenintake();
//            rbg.pmove(12,true);
//            rbg.aspecimen_outtake();
//            rbg.pmove(13,true);
//            rbg.stop_drive();
//            rbg.armRotate.setPower(0);
//             rbg.armRotateLeft.setPower((0));
//             sleep(2000);
//
////            rbg.rotatetargetPIDF(0);
////
////            rbg.stop_drive();
////            rbg.armRotate.setPower(-0.12);
////            rbg.armRotateLeft.setPower((0.12));
////            rbg.pause(200);
////            rbg.armRotate.setPower(0);
////            rbg.armRotateLeft.setPower((0));
////            sleep(1000000000);
//
//
////
////
////            rbg.updatePoseEstimate();
////            telemetry.addData("x", rbg.pose.position.x);
////            telemetry.addData("y", rbg.pose.position.y);
////            telemetry.addData("heading", rbg.imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES)));
////            telemetry.update();
////            //outtake
////            rbg.delay(100000000);
////
////
////
////
////            //intake
////            rbg.pmove(8,true);//strafe
////            //outake
////            rbg.pmove(9,true);//strafe
////            //intake
////            rbg.pmove(9,true);//strafe
////            //outake;
////
////
//////            rbg.move(-0.3);
//////            rbg.delay(600); //800
//////            rbg.stop_drive();
////            rbg.aspecimen_outtake();
////            rbg.movecontrol(45);
////            rbg.aspicmenintake();
////            rbg.movecontrol(-48);
////          //  rbg.delay(1100);
////            rbg.aspecimen_outtake();
////
////            rbg.movecontrol(55);
////            rbg.stop_drive();
////            rbg.delay(100);
////            rbg.rotatetargetPIDF(0);
////         //   rbg.delay(400);
////            rbg.move(0.3);
////            rbg.delay(450);
////
////            rbg.stop_drive();
////            rbg.armRotate.setPower(-0.12);
////            rbg.armRotateLeft.setPower((0.12));
////            rbg.pause(400);
////            rbg.armRotate.setPower(0);
////            rbg.armRotateLeft.setPower((0));
////            sleep(1000000000);
//
//
//        }
//    }
//}