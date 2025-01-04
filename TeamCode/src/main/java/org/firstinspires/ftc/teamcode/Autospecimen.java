package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoSpecimen", group = "A")
//@Disabled
@Config
public class Autospecimen extends LinearOpMode {
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

        if (gamepad1.triangle){rbg.field = 1;}

        }
        rbg.init(3);
        telemetry.addData(">", "Press Play to start op mode ");
        rbg.updatePoseEstimate();
        telemetry.addData("x",rbg.pose.position.x);
        telemetry.addData("y",rbg.pose.position.x);
        telemetry.update();
        while(!isStarted())
        {
            rbg.delay(25);
        }
        waitForStart();

        while (opModeIsActive()) {
            rbg.afirst_spec_outake();
            rbg.afmove(0,true);//strafe samples
         //   rbg.aspec_intake();;


           // rbg.delay(1000000);
            rbg.afmove(1,false);// forward for sampels


            rbg.afmove(2,true);//strafe for first samples
            rbg.afmove(3,false);//push the first sample

            rbg.afmove(4,false);//forward for second sample
            rbg.afmove(5,true);//strafe for second sampl
            rbg.afmove(6,false);//push the seond samples and intake

            rbg.aspec_intake();
            rbg.afmove(7,true);//strafe for outtake
            rbg.aspec_outtake();;
            rbg.afmove(8,true);//strafe for intake
            rbg.aspec_intake();;
            rbg.afmove(9,true);//strafe for outtake
            rbg.aspec_outtake();;
            rbg.afmove(10,true);//strafe for intake;
            rbg.aspec_intake();;
            rbg.afmove(11,true);//strafe for outtake;
            rbg.flag[rbg.last]=true;
            rbg.aspec_outtake();;
            rbg.afmove(12,true);//strafe for park;

            rbg.stop_drive();;//strafe for park;
//            rbg.armRotateLeft.setPower(0);
//            rbg.armRotate.setPower(0);
            //  rbg.armRotateLeft.setPower((0));
            sleep(50000);


        }
    }
}