package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "TelMeet1", group = "AA")
public class TelOpB extends LinearOpMode {
    BaseClass rbg;// hardware init at Mecanumdrive.
    double speed_factor = 1.0;
    public enum State {
        INTAKE,
        LIFT,
        OUTTAKE,
    }

    State state = State.INTAKE;

    @Override
    public void runOpMode() {
        Pose2d p1 = new Pose2d(0, 0, 0);
        rbg = new BaseClass(this, p1);
        if (isStopRequested()) return;
        telemetry.addLine("Wait ! Initializing............. ");
        telemetry.addLine("Do not press start  ");
        telemetry.update();
        sleep(2000);
        telemetry.addLine("Press Start Now!:");
        telemetry.update();
        waitForStart();
       rbg.timer(0,rbg.start);
        rbg.init(0);
        state= State.INTAKE;
        while (opModeIsActive()) {
            switch (state) {
                case INTAKE:
                    if(gamepad2.right_bumper) {
                        speed_factor = 0.3;
                        rbg.intake_shift(false);
                    }

                    if (gamepad2.left_bumper){
                        speed_factor = 0.3;
                        rbg.specimen_pre();
                    }

                    if(gamepad1.right_bumper )  {
                        if (!rbg.flag[rbg.specimen]) rbg.sample_intake();
                        else rbg.specimen_intake();
                        speed_factor = 1.0;
                        state= State.LIFT;
                    }
                    break;
                case LIFT:

                    if (gamepad1.right_bumper|| rbg.flag[rbg.lift]) {
                        if (!rbg.flag[rbg.specimen]){
                            if (rbg.sample_lift(1)) {
                                speed_factor = 0.3;
                                state = State.OUTTAKE;
                            }
                        }
                        else {
                            if (rbg.specimen_lift(1)) {
                                speed_factor = 0.3;
                                state = State.OUTTAKE;
                            }
                        }

                    }
                    if(gamepad1.left_bumper) {
                        rbg.intake_drop();
//                        rbg.flag[rbg.lift] = false;
                        speed_factor = 0.3;
                        state= State.INTAKE;
                    }
                    break;

                case OUTTAKE:

                    if (gamepad1.right_bumper||rbg.flag[rbg.outtake])
                    {
                        if (!rbg.flag[rbg.specimen]){
                            if(rbg.sample_outtake(1)) {
                                speed_factor = 1.0;
                                state = State.INTAKE;
                            }
                        }
                        else {
                            if(rbg.specimen_outtake(1)) {
                                speed_factor = 1.0;
                                state = State.INTAKE;
                            }

                        }




                    }
                    break;


            }



              if(gamepad1.share) {

                rbg.hang();// 2 times.
              }
              if(gamepad2.ps) rbg.flag[rbg.force]=true;
            rbg.adjust(gamepad2.dpad_up,gamepad2.dpad_down);
              rbg.robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, speed_factor);


//            telemetry.addData("TempInput", rbg.tempinput);
//            telemetry.update();

//            if (rbg.driving) {
//                if (robo_drive)
//                    rbg.robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
//                else
//                    rbg.field_centric(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
//            }

            telemetry.addData("flag",rbg.flag[rbg.offset]);
            telemetry.update();
            }
        }
    }










