package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AutoRegional", group = "A")
//@Disabled
@Config
public class Auto extends LinearOpMode {
    // private ElapsedTime runtime = new ElapsedTime();


    Pose2d pp = new Pose2d(0, 0, 0);
    BaseClass rbg;
   // Pose2d start = new Pose2d(0, 0, Math.toRadians(0));


    @Override
    public void runOpMode() {
        rbg = new BaseClass(this, pp);
        rbg.init(0);

        telemetry.update();
        sleep(500);
        rbg.init(2);//rest the roatioan 0 positon
        sleep(500);
        telemetry.addLine("Make sure the arm was already placed  on the right location before initialization");
        telemetry.addLine("Now Push the linear slide all the way down , turn the arm to the target position and load the specimen.");
        telemetry.addLine("Aftr that, hold the arm and press gunner gamepad right bumper.");
        telemetry.update();
        while(!gamepad1.right_bumper&&opModeIsActive())
        {
        sleep(100);
        }
        rbg.init(3);
        sleep(500);
        telemetry.addData(">", "Press Play to start op mode ");
        telemetry.update();
        waitForStart();
        rbg.init(4);
        while (opModeIsActive()) {

            rbg.move(0.3);
            rbg.delay(150);

            rbg.pidfsetting(rbg.rotate_spec_out,rbg.pidf_outtake_spec);//pre postion   rbg.Intake_handle.setPosition(rbg.handle_specimen_outtake);
            rbg.delay(500);
            rbg.Intake_handle.setPosition(rbg.handle_specimen_outtake);
            rbg.linearslide(rbg.slide_idle, rbg.slidev2);


            rbg.aspec_outtake();
            //   rbg.rotate[rbg.soutb]=4000;
            //   rbg.updatePoseEstimate();
            rbg.afmove(0,true);//strafe samples
            rbg.afmove(1,false);// forward for sampels
            rbg.afmove(2,true);//strafe for first samples
            rbg.afmove(3,false);//push the first sample
//            rbg.updatePoseEstimate();
//            telemetry.addData("3x", rbg.pose.position.x);
//            telemetry.addData("y", rbg.pose.position.y);
//            telemetry.addData("heading", rbg.imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES)));
//            telemetry.update();

            rbg.afmove(4,false);//forward for second sample
//            telemetry.addData("4x", rbg.pose.position.x);
//            telemetry.addData("y", rbg.pose.position.y);
//            telemetry.addData("heading", rbg.imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES)));
//            telemetry.update();
            rbg.afmove(5,true);//strafe for second sampl
            rbg.afmove(6,false);//push the seond samples and intake
            // rbg.delay(200000000);
            rbg.aspec_outtake();;
            rbg.afmove(7,true);//strafe for outtake
            rbg.aspec_outtake();;
            rbg.afmove(8,true);//strafe for intake
            rbg.aspec_outtake();;
            rbg.afmove(9,true);//strafe for outtake
            rbg.aspec_outtake();;
            rbg.afmove(10,true);//strafe for intake;
            rbg.aspec_outtake();;
            rbg.afmove(11,true);//strafe for outtake;
            rbg.aspec_outtake();;
//            rbg.armRotate.setPower(0);
//            rbg.armRotateLeft.setPower(0);
            rbg.rotateTarget=0;
            rbg.afmove(12,true);//strafe for park;
            rbg.stop_drive();;//strafe for park;
//            rbg.armRotateLeft.setPower(0);
//            rbg.armRotate.setPower(0);
            //  rbg.armRotateLeft.setPower((0));
            sleep(50000);


        }
    }
}