package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "AutoSamples", group = "A")
//@Disabled
@Config
public class Autosample extends LinearOpMode {
    // private ElapsedTime runtime = new ElapsedTime();


    Pose2d pp = new Pose2d(0, 0, 0);
    BaseClass rbg;
   // Pose2d start = new Pose2d(0, 0, Math.toRadians(0));


    @Override
    public void runOpMode() {
        rbg = new BaseClass(this, pp);
        rbg.init(0);
        rbg.baserest=true;

        telemetry.update();
        sleep(500);
        rbg.init(2);//rest the roatioan 0 positon
        sleep(500);
        rbg.updatePoseEstimate();
        telemetry.addData("x",rbg.pose.position.x);
        telemetry.addData("y",rbg.pose.position.y);
        telemetry.addLine("Make sure the arm was already placed  on the right location before initialization");
        telemetry.addLine("Now Push the linear slide all the way down , turn the arm to the target position and load the specimen.");
        telemetry.addLine("Aftr that, hold the arm and press gunner gamepad right bumper.");
        telemetry.update();
        while(!isStopRequested())
        {
        sleep(20);
        if(gamepad1.right_bumper) break;
        }
        rbg.init(3);
        telemetry.addData(">", "Press Play to start op mode ");
        rbg.updatePoseEstimate();
        telemetry.addData("x",rbg.pose.position.x);
        telemetry.addData("y",rbg.pose.position.y);
        telemetry.update();
        while(!isStarted())
        {
            rbg.delay(25);
        }
        waitForStart();

        rbg.timer(0,8);
        while (opModeIsActive()) {

           rbg.asamplefirstmove();
            rbg.amove(0); //preload sample move to outake
            rbg.asample_outtake();
            rbg.amove(1);// forward to 1st sample for intake;
            rbg.asample_intake();
         //   rbg.delay(300);
//            rbg.updatePoseEstimate();
//            telemetry.addData("x0",rbg.xo);
//            telemetry.addData("y0",rbg.yo);
//            telemetry.addData("a0",rbg.ao);
//
//            telemetry.addData("x",rbg.pose.position.x);
//            telemetry.addData("y",rbg.pose.position.y);
//            telemetry.addData("a",rbg.imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES)));
//            telemetry.update();
//            rbg.delay(1000000000);
            rbg.amove(2);
            rbg.asample_outtake();
            rbg.amove(3);
            rbg.asample_intake();
            rbg.amove(4);
            rbg.asample_outtake();
            rbg.amove(5);
            rbg.flag[rbg.last]=true;
            rbg.asample_intake();
            rbg.flag[rbg.last]=false;
            rbg.amove(6);
            rbg.asample_outtake();
            rbg.delay(300);
            rbg.updatePoseEstimate();
            rbg.delay(200);
            telemetry.addData("x0",rbg.xo);
            telemetry.addData("y0",rbg.yo);
            telemetry.addData("a0",rbg.ao);

            telemetry.addData("x",rbg.pose.position.x);
            telemetry.addData("y",rbg.pose.position.y);
            telemetry.addData("a",rbg.imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES)));
            telemetry.update();
            rbg.delay(1000000000);



            // Specific to last sample

           // rbg.flag[rbg.ain]=true;
           // rbg.rotatetargetPIDF(rbg.rotate[rbg.pain3]); // preintake arm pos
            //rbg.linearslide(rbg.armSlide, rbg.slide[rbg.pain3], rbg.armslideV3);
           rbg.armrotatePIDF();
            rbg.delay(250);
          //  rbg.flag[rbg.ain]=true;

          //  rbg.rotatetargetPIDF(rbg.rotate[rbg.ain3]); //intake arm pos value
            rbg.armrotatePIDF();
            rbg.delay(200);

          //  rbg.Claw.setPosition(rbg.clawClose);
            rbg.delay(350); // 300
        //    rbg.linearslide(rbg.armSlide, rbg.slide[rbg.idle], rbg.armslideV3);

            rbg.delay(150);

          //  rbg.Handle.setPosition(rbg.handleOut1);
          //  rbg.rotatetargetPIDF(rbg.rotate[rbg.idle]);
            rbg.delay(150);
            rbg.amove(6);

            rbg.asample_outtake();
         //   rbg.flag[rbg.lasta] = true;
            rbg.move(0.3);
         //   rbg.linearslide(rbg.armSlide,rbg.slide[rbg.aend],rbg.armslideV3);
            // rbg.Handle.setPosition();
            rbg.delay(100);
            // rbg.stop_drive();


          //  rbg.rotatetargetPIDF(rbg.rotate[rbg.aend]);
            rbg.delay(100);
            //    rbg.stop_drive();

            rbg.amove(7);
            rbg.move(0.45);
          //  rbg.linearslide(rbg.armSlide,1000,2000);
           // double prey=0; rbg.flagreset=true;
            while(opModeIsActive()&&!rbg.timer(29800,8)) {

                rbg.armrotatePIDF();
                rbg.updatePoseEstimate();
            //    if (Math.abs(rbg.pose.position.y - prey) < 0.01) break;
             //   prey=rbg.pose.position.y;
            }
         //   rbg.armRotate.setPower(0);
          //  rbg.armRotateLeft.setPower(0);
            rbg.stop_drive();
            sleep(100000);

        }
    }
}