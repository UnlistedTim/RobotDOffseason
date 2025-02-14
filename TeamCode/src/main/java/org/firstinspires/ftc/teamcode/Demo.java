package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleopDemo", group = "AA")
public class Demo extends LinearOpMode {
    BaseClass rbg;// hardware init at Mecanumdrive.
    double speed_factor = 1.0;

    public enum State {
        IDLE,
        SAMPLEINTAKE,
        INTAKEIDLE,
        SPECINTAKE,
        SAMPLELIFT,
        SAMPLEOUTTAKE,
        SPECOUTTAKE

    }

    State state = State.IDLE;
    @Override
    public void runOpMode() {
        Pose2d p1 = new Pose2d(0, 0, 0);
        rbg = new BaseClass(this, p1);
        boolean flagpreintake=false;
        if (isStopRequested()) return;
        telemetry.addLine("Wait ! Initializing............. ");
        telemetry.addLine("Do not press start  ");
        telemetry.update();
        rbg.init(0);
        rbg.init(2);
        sleep(500);
        telemetry.addLine("Press Start Now!:");
        telemetry.update();



//
        waitForStart();
      //  rbg.pp0=new Pose2d(0, 0, 0);
        rbg.timer(0,rbg.start);
        rbg.pidf_index = rbg.pidf_idle_demo;
        rbg.pidfsetting(rbg.arm_angle_preintake);
        rbg.delay(500);

        rbg.Left_handle.setPosition(rbg.lefthandle_intake);
        rbg.Right_handle.setPosition(rbg.righthandle_intake);

        state = State.SAMPLEINTAKE;
        while (opModeIsActive()) {


 switch (state) {
 case SAMPLEINTAKE:

                    rbg.intake_claw_rotate(gamepad1.left_stick_x);
                   if(gamepad1.right_bumper) {
                       rbg.pidf_index=rbg.pidf_idle_demo;
                       rbg.pidfsetting(rbg.arm_arngle_intake);
                       rbg.delay(180); // 500;
                       rbg.Claw.setPosition(rbg.claw_close);
                       rbg.delay(350);
                       rbg.Left_handle.setPosition(rbg.lefthandle_idle);
                       rbg.Right_handle.setPosition(rbg.righthandle_idle);
                       rbg.delay(300);
                       state = State.SAMPLEOUTTAKE;
                       break;
                   }
                   break;

//                case INTAKEIDLE:
//
//                   if(gamepad1.right_bumper) {
//                       rbg.pre_specintake(false);
//                       rbg.pidf_index=rbg.pidf_idle_specin;
//                       speed_factor=1;
//                       state = State.SPECINTAKE;
//                       break;
//                   }
//                  rbg.intake_smooth_shift(gamepad2.right_stick_y);
//                  rbg.intake_claw_rotate(gamepad2.left_stick_x);
//                  break;

  case SPECINTAKE:


                  if(gamepad1.right_bumper) {
                      rbg.pidf_index=rbg.pidf_outtake_up2;
                      rbg.pidfsetting(rbg.arm_angle_specintake);
                      rbg.Left_handle.setPosition(rbg.lefthandle_specintake);
                      rbg.Right_handle.setPosition(rbg.righthandle_specintake);
                      rbg.delay(1500);
                      state = State.SPECOUTTAKE;
                  }

                break;




 case SAMPLEOUTTAKE:




                 if(gamepad1.right_bumper) {

                     rbg.pidf_index=rbg.pidf_idle_sampleout;
                     rbg.pidfsetting(rbg.arm_angle_sampleouttake);
                     rbg.Left_handle.setPosition(rbg.lefthandle_sampleouttake);
                     rbg.Right_handle.setPosition(rbg.righthandle_sampleouttake);
                        rbg.delay(2000);
                     rbg.Claw.setPosition(rbg.claw_open);
                     rbg.delay(300);
                     state = State.SPECINTAKE;

                 }

                 break;


  case SPECOUTTAKE:




                   if(gamepad1.right_bumper) {
                       if (rbg.arm_angle >= 197 && rbg.arm_angle <= 209){
                           rbg.Left_handle.setPosition(rbg.In_LHandle.get(rbg.arm_angle));
                           rbg.Right_handle.setPosition(rbg.In_RHandle.get(rbg.arm_angle));
                       }
                       rbg.delay(2000);
                       rbg.Claw.setPosition(rbg.claw_close);
                       rbg.delay(1000);
                       rbg.pidf_index=rbg.pidf_specin_specout;
                       rbg.pidfsetting(rbg.arm_angle_specouttake);
                       rbg.Left_handle.setPosition(rbg.lefthandle_specouttake);
                       rbg.Right_handle.setPosition(rbg.righthandle_specouttake);
                       rbg.delay(1500000000);

                   }

                   break;

            }



            rbg.armrotatePIDF();

            }
        }
    }











