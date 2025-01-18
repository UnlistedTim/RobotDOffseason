package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "TelopState A", group = "AA")
public class TelOp extends LinearOpMode {
    BaseClass rbg;// hardware init at Mecanumdrive.
    double speed_factor = 1.0;
  //  double rotpowerl=0,rotpowerr=0;


    public enum State { // linear slide 600, arm 1500, rot 0.2
        IDLE,
        SAMPLEINTAKE,
        INTAKEIDLE,
        SPECIMENINTAKE,
        SPECINTAKE,
        SAMPLELIFT,
        SAMPLEOUTTAKE,
        SPECOUTTAKE,


    }

    State state = State.IDLE;




    @Override
    public void runOpMode() {
        Pose2d p1 = new Pose2d(0, 0, 0);
        rbg = new BaseClass(this, p1);
        if (isStopRequested()) return;
        telemetry.addLine("Wait ! Initializing............. ");
        telemetry.addLine("Do not press start  ");
        telemetry.update();
        rbg.init(0);
        rbg.init(1);
        sleep(500);
        telemetry.addLine("Press Start Now!:");
        telemetry.update();
        waitForStart();
        rbg.pre_idle();
     //   rbg.rotation_reset();

        while (opModeIsActive()) {
            switch (state) {
               case IDLE:
                   if(!rbg.flag[rbg.idleready])
                   {
                       rbg.idle_ready();
                       if(speed_factor<1)speed_factor=rbg.speed_index;// ater sample outtake
                       break;
                   }
                   if(gamepad2.right_bumper) {
                       rbg.pre_sampleintake();
                       state = State.SAMPLEINTAKE;
                       break;

                   };
                   if(gamepad2.left_bumper) {
                       rbg.pre_specintake(false);
                       state = State.SPECINTAKE;
                       break;

                   };



               case SAMPLEINTAKE:
                   if(!rbg.flag[rbg.sampleintakeready])
                   {
                       speed_factor=0.3;
                       rbg.sampleintake_ready(gamepad2.right_bumper);
                       break;
                   }

                   if(gamepad2.right_bumper) {
                       rbg.sampleintake();
                       speed_factor=1;
                       state = State.INTAKEIDLE;
                       break;
                   }

                   if(gamepad2.left_bumper) {
                       rbg.pre_specintake(false);
                       speed_factor=1;
                       state = State.SPECINTAKE;
                       break;



                   }

                  rbg.intake_smooth_shift(gamepad2.right_stick_y);
                   rbg.intake_claw_rotate(gamepad2.left_stick_x);

                  break;

                case INTAKEIDLE:
                    if(!rbg.flag[rbg.intakeidleready])
                    {
                        rbg.intakeidle_ready();
                        break;
                    }
                    if(gamepad2.right_bumper) {
                        rbg.pre_samplelift(false);
                        state = State.SAMPLELIFT;
                        break;
                    };

                    if(gamepad2.left_bumper) {
                        rbg.pre_specintake(false);
                      state = State.SPECINTAKE;
                        break;
                    };

                    if(gamepad1.right_bumper) {
                        rbg.pre_samplelift(true);
                        state = State.SAMPLELIFT;
                        break;
                    };


                break;

               case SPECINTAKE:
                   if(!rbg.flag[rbg.specintakeready])
                   {
                       rbg.specintake_ready();
                       break;
                   }
                   if(gamepad2.right_bumper) {
                       rbg.linearslide(rbg.slide_idle,rbg.slidev2 );
                       rbg.pre_samplelift(false);
                       state = State.SAMPLELIFT;
                       break;
                   };


                   if(gamepad1.touchpad||rbg.flag[rbg.placement])
                   {
                       rbg.specplacment();
                       speed_factor=0.4;
                   }
                   if(gamepad1.right_bumper)
                   {
                       rbg.specintake();
                       rbg.pre_specouttake();
                       speed_factor=1;

                   }



                   break;

                case SAMPLELIFT:
                   if(!rbg.flag[rbg.sampleouttakeready])
                   {
                      if(!rbg.flag[rbg.lift] && gamepad1.right_bumper)  rbg.flag[rbg.lift]=true;
                      rbg.samplelift_ready();
                       break;
                   }
                if(gamepad1.right_bumper||rbg.flag[rbg.lift])
                {
                   rbg.pre_sampleouttake();
                   speed_factor=0.3;
                   state = State.SAMPLEOUTTAKE;
                }
                if(gamepad2.left_bumper) {
                    rbg.pre_specintake(false);
                    state = State.SPECINTAKE;
                    break;
                    };


               break;

                case SAMPLEOUTTAKE:
                    if(!rbg.flag[rbg.sampleouttakeready])
                    {
                        rbg.sampleouttake_ready();
                        break;
                    }

                 if(gamepad1.right_bumper)
                 {
                     rbg.sampleouttake();
                     rbg.pre_idle();

                     state = State.IDLE;
                     break;
                 }

                 break;


               case SPECOUTTAKE:
                   if(!rbg.flag[rbg.specouttakeready])
                   {
                       rbg.specouttake_ready();
                       break;
                   }

                   if(gamepad1.right_bumper) {
                       rbg.specouttake();
                       rbg.pre_idle();
                       state = State.IDLE;

                   }


                   break;






            }


            rbg.armrotatePIDF();

            if(gamepad1.left_bumper) {
               if (rbg.drop())  state = State.IDLE;
               else state = State.SAMPLEINTAKE;
            }

            if (rbg.timer(88000, rbg.start) || rbg.flag[rbg.force]) {
                if (gamepad2.share || rbg.flag[rbg.hang]) {
                    rbg.pre_hang();
                }
                if (gamepad1.share) {
                   rbg.hang();
                 //   k = k/4;
                  //  rbg.pidfsetting(1500,rbg.pidf_hang3); // Hit arm with low rung
                   // rbg.delay(300);
                }

            }


            if (gamepad2.ps) rbg.flag[rbg.force] = true;


            rbg.robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, speed_factor);

//            if(gamepad1.ps) {
//                state = State.RESTMODE;
//            }


             telemetry.addData("armlinerslide top", rbg.Slide_top.getCurrentPosition());
            telemetry.addData("armlinerslide bot", rbg.Slide_bot.getCurrentPosition());
            telemetry.addData("armrotate position", -rbg.Arm_right.getCurrentPosition());


            telemetry.addData("LEFT power", rbg.Arm_left.getPower());




//            telemetry.addData("Intake HANDLE", rbg.  Intake_handle.getPosition());
//            telemetry.addData("Intake rot", rbg.Intake_rot.getPosition());
           telemetry.addData("gerbox", rbg.Gearbox.getPosition());
            telemetry.addData("intake_lvel", rbg.intake_level);
            telemetry.addData("Right arm motor current", rbg.Arm_right.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Left arm motor current", rbg.Arm_left.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("Top slide motor current", rbg.Slide_top.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Bot slide motor current", rbg.Slide_bot.getCurrent(CurrentUnit.AMPS));

            telemetry.update();
            }
        }
    }











