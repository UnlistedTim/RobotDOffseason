//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//@Autonomous(name = "Automeet2", group = "A")
////@Disabled
//@Disabled
//@Config
//public class Autoback extends LinearOpMode {
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
//            rbg.pmove(1,true);//strafe
//            rbg.pmove(2,false);//forward
//            rbg.pmove(3,true);//strafe
//
//
//            rbg.updatePoseEstimate();
//            telemetry.addData("x", rbg.pose.position.x);
//            telemetry.addData("y", rbg.pose.position.y);
//            telemetry.addData("heading", rbg.imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES)));
//            telemetry.update();
//            //outtake
//            rbg.delay(100000000);
//
//            rbg.pmove(4,false);//foward
//            rbg.pmove(5,false);//backware
//            rbg.pmove(6,true);//strafe
//            rbg.pmove(7,false);//foward
//            //intake
//            rbg.pmove(8,true);//strafe
//            //outake
//            rbg.pmove(9,true);//strafe
//            //intake
//            rbg.pmove(9,true);//strafe
//            //outake;
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//            rbg.move(-0.3);
//            rbg.delay(600); //800
//            rbg.stop_drive();
//            rbg.aspecimen_outtake();
//            rbg.move(0.3);
//            rbg.delay(300);
//            rbg.stop_drive();
//            rbg.delay(100);
//            rbg.movecontrol(45);
//            rbg.stop_drive();
//            rbg.delay(100);
//
//            rbg.aspicmenintake();
//            rbg.move(-0.3);
//            rbg.delay(400);
//            rbg.stop_drive();
//            rbg.delay(100);
//            rbg.movecontrol(-48);
//          //  rbg.delay(1100);
//            rbg.aspecimen_outtake();
//            rbg.move(0.3);
//            rbg.delay(300);
//            rbg.stop_drive();
//            rbg.delay(100);
//            rbg.movecontrol(55);
//            rbg.stop_drive();
//            rbg.delay(100);
//            rbg.rotatetargetPIDF(0);
//            rbg.delay(400);
//            rbg.move(0.3);
//            rbg.delay(450);
//
//            rbg.stop_drive();
//            rbg.armRotate.setPower(-0.13);
//            rbg.armRotateLeft.setPower((0.13));
//            rbg.pause(400);
//            rbg.armRotate.setPower(0);
//            rbg.armRotateLeft.setPower((0));
//            sleep(1000000000);
//
//
//        }
//    }
//}