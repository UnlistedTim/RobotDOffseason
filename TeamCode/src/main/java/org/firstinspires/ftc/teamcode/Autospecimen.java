package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoSpec", group = "A")
//@Disabled
@Config
public class Autospecimen extends LinearOpMode {
    // private ElapsedTime runtime = new ElapsedTime();


    Pose2d pp = new Pose2d(0, 0, 0);
    BaseClass rbg;


    @Override
    public void runOpMode() {
        rbg = new BaseClass(this, pp);
        telemetry.addLine("Make sure linearslide was fully in befroe start and do not move the robot anymore;");
        telemetry.update();
        rbg.init(0);
        sleep(500);
        rbg.init(1);
        telemetry.addLine("Preload the specimen in the claw and press Driver right bumper;");
        telemetry.update();
        while(!isStopRequested())
        {
            sleep(20);
            if(gamepad1.right_bumper){
             rbg.Claw.setPosition(rbg.claw_close);
             break;
            }
        }

        sleep(200);
        rbg.updatePoseEstimate();
        telemetry.addData("x",rbg.pose.position.x);
        telemetry.addData("y",rbg.pose.position.y);
        telemetry.addData(">", "Press Play to start op mode ");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            rbg.delay(4000);

            rbg.aspec_outtake();
            rbg.afmove(0,true);//strafe samples

            rbg.afmove(1,false);// forward for sampels


            rbg.afmove(2,true);//strafe for first samples
            rbg.afmove(3,false);//push the first sample

            rbg.afmove(4,false);//forward for second sample
            rbg.afmove(5,true);//strafe for second sampl
            rbg.afmove(6,false);//push the seond samples and intake
            rbg.afmove(20,false);// forward for 3rd
            rbg.afmove(21,true);//strafe  for 3rd

            rbg.pidf_index=rbg.pidf_specintake;
            rbg.pidfsetting(rbg.arm_angle_specintake);


            rbg.afmove(22,false);// push for 3 rd
            rbg.stop_drive();
//            rbg.delay(10000000);
            rbg.aspec_intake();
            rbg.afmove(7,true);//strafe for outtake

//            rbg.delay(100000);
            rbg.aspec_outtake();;
            rbg.afmove(8,true);//strafe for intake
            rbg.aspec_intake();;
            rbg.afmove(9,true);//strafe for outtake
            rbg.aspec_outtake();;
            rbg.afmove(10,true);//strafe for i ntake;
            rbg.aspec_intake();
            rbg.afmove(11,true);//strafe for outtake;
            rbg.flag[rbg.last]=true;
            rbg.aspec_outtake();
            rbg.afmove(12,true);//strafe for park;

            rbg.stop_drive();;//strafe for park;
//            rbg.armRotateLeft.setPower(0);
//            rbg.armRotate.setPower(0);
            //  rbg.armRotateLeft.setPower((0));
            sleep(50000);


        }
    }
}