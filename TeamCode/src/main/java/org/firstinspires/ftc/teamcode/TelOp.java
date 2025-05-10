package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "RobotDTeleop", group = "AA")
public class TelOp extends LinearOpMode {
    BaseClass rbg;// hardware init at Mecanumdrive.
    double speed_factor = 1.0;

//    double[] deltaSlidePowers= new double[1000];
//
//    double deltaPower = 0;

//    int index = 0;



    public enum State {
        IDLE,
        SAMPLEINTAKE,
        INTAKEIDLE,
        SPECINTAKE,
        SAMPLELIFT,
        SAMPLEOUTTAKE,
        SPECOUTTAKE,
        HANG

    }

    State state = State.IDLE;
    @Override
    public void runOpMode() {
        Pose2d p1 = new Pose2d(0, 0, 0);
        rbg = new BaseClass(this, p1);
        if (isStopRequested()) return;
        telemetry.addLine("Please wait, Currently Initializing............. ");
        telemetry.addLine("Do NOT press start");
        telemetry.update();
        rbg.init(0);
        rbg.init(2);
        sleep(500);
        telemetry.addLine("Press Start Now!: ");
        telemetry.addLine(":D");

        telemetry.update();
        rbg.pidf_index=rbg.pidf_idle;


//        rbg.Front_led.setPosition(0.5);
//        rbg.Back_led.setPosition(0.5);
        waitForStart();
      //  rbg.pp0=new Pose2d(0, 0, 0);
        while (opModeIsActive())

            switch (state) {
               case IDLE:

                   break;

case SAMPLEINTAKE:

                  break;
case INTAKEIDLE:

                break;

case SPECINTAKE:

                break;

  case SAMPLELIFT:

               break;


 case SAMPLEOUTTAKE:


                 break;


  case SPECOUTTAKE:

                   break;



    case HANG:


                break;

        }


            if (gamepad2.ps) rbg.flag[rbg.force] = true;


            rbg.robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, speed_factor);


            }
        }











