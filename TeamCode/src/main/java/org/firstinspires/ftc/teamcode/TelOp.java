package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "TelMeet2", group = "AA")
public class TelOp extends LinearOpMode {
    BaseClass rbg;// hardware init at Mecanumdrive.
    double speed_factor = 1.0;

    public enum State {
        INTAKE,
        SPECIMENINTAKE,
        LIFT,
        OUTTAKE,
        SPECIMENOUTTAKE;
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
        rbg.init(0);
        sleep(500);
        telemetry.addLine("Press Start Now!:");
        telemetry.update();
        waitForStart();

        rbg.timer(0, rbg.start);
        rbg.Slide_bot.setTargetPosition(0);
        rbg.Slide_bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbg.Slide_bot.setVelocity(0);


        rbg.Slide_top.setTargetPosition(0);
        rbg.Slide_top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbg.Slide_top.setVelocity(0);
        rbg.Arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbg.Arm_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rbg.Arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbg.Arm_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // rbg.Intake_rot.setPosition(0.3);
        //rbg.Intake_rot.setPosition(0.3);
        sleep(500);


        rbg.init(1);
      //  state = State.INTAKE;

        while (opModeIsActive()) {
            switch (state) {
                case INTAKE:
                  //  if(gamepad2.left_bumper) {state = State.SPECIMENINTAKE;break;}
                    if (gamepad2.right_bumper&&rbg.flag[rbg.intake_ready]&& rbg.flag[rbg.button_flip]) {
                        rbg.intake();
                        if (rbg.flag[rbg.color_check]){
                            state=State.LIFT;
                            speed_factor=1.0;
                            break;
                        }

                    }
                    if(!rbg.flag[rbg.button_flip]&&rbg.flag[rbg.intake_ready]&&!gamepad2.right_bumper)
                    {
                        rbg.flag[rbg.button_flip]=true;// prevent gunner hold the right bumper.
                    }
                    if(!rbg.flag[rbg.intake_ready] &&gamepad2.right_bumper){
                        //rbg.pre_intake();
                        rbg.pre_intake();
                        speed_factor=0.4;

                }

                    if (rbg.flag[rbg.intake_ready]) {
                       rbg.intake_shift(gamepad2.right_stick_y,false);
                        rbg.pre_intake_adjust(gamepad2.left_stick_x,gamepad2.left_stick_y);
                    }






//
//                    if (gamepad1.right_bumper) {
//                        if (!rbg.flag[rbg.specimen]) {
//                            if (rbg.sample_intake()) {
//                                state = State.LIFT;
//                                speed_factor = 1.0;
//                            }
//                        } else {
//                            if (rbg.specimen_intake()) {
//                                speed_factor = 1.0;
//                                state = State.OUTTAKE;
//                                break;
//                            }
//                        }
//
//                    }
                    break;

                case SPECIMENINTAKE:

                    break;


                case LIFT:

                    if (gamepad1.right_bumper || rbg.flag[rbg.lift]) {
                        if (rbg.lift()) {
                            speed_factor = 0.3;
                            state = State.OUTTAKE;
                            break;
                        }

                    }
                    if (gamepad1.left_bumper) {
                        rbg.intake_drop();
                        state = State.INTAKE;
                        speed_factor=0.4;
                        break;
                    }

                    if (gamepad1.touchpad) {
                        rbg.intake_throw();
                        rbg.flag[rbg.intake_shift] = false;
                        speed_factor = 1.0;
                        state = State.INTAKE;
                    }
                    break;

                case OUTTAKE:

                    if (gamepad1.right_bumper || rbg.flag[rbg.outtake]) {

                            if (rbg.outtake()) {
                                speed_factor = 1.0;
                                state = State.INTAKE;
                            }


                    }

                    break;
//                    if (rbg.flag[rbg.specimen] && gamepad1.left_bumper) {
//                        rbg.specimen_drop();
//                        speed_factor = 1.0;
//                        state = State.INTAKE;
//
//                    }


                case SPECIMENOUTTAKE:
                    break;

            }

            // jerome test
            //if(gamepad1.share) rbg.hang();// 2 times.
//            if (gamepad1.share) rbg.hang();
//            if (gamepad1.dpad_down) {
//                rbg.stop_drive();
//                sleep(500);
//                if (gamepad1.dpad_down) {
//                    rbg.stop_drive();
//                    rbg.Handle.setPosition(rbg.handleIdle);
//                    sleep(300);
//                    rbg.reset();
//                }
//            }

            rbg.armrotatePIDF();

            if (gamepad2.share||rbg.flag[rbg.hang]){

                rbg.pre_hang();
            }

            if (gamepad1.share){

                telemetry.update();
                //rbg.hang();
                //rbg.stop_drive();



               // rbg.pidfsetting(700, rbg.pidf_hang_up);


               // rbg.delay(750);
//                rbg.Gearbox.setPosition(0.95); // High torque
//                rbg.delay(500);
//                rbg.linearslideTq(1000,rbg.slidev1);
//
//                rbg.delay(750); // 1000
//                rbg.Intake_handle.setPosition(0.7);
//                rbg.linearslideTq(4700,rbg.slidev2);
//                rbg.delay(1750);
                if(!rbg.flag[rbg.hang0]) break;
                while (rbg.Slide_top.getCurrentPosition() < 4500 && opModeIsActive())
                {
                    rbg.delay(25);
                }
                rbg.k = 0.000035/2;
                rbg.pidfsetting(1425, rbg.pidf_hang3); // Hit arm with low rung
                rbg.delay(1000);
                rbg.linearslideTq(4000,0.95);
               // rbg.delay(1000); //1500
                rbg.pidfsetting(2000, rbg.pidf_hang2); //1600
              //  rbg.delay(1000);
                rbg.linearslideTq(-500,0.95);

                telemetry.addData("linear slide top value", rbg.Slide_top.getCurrentPosition());
                telemetry.addData("linear slide bot value", rbg.Slide_bot.getCurrentPosition());
                telemetry.addData("linear slide top current", rbg.Slide_top.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("linear slide bot current", rbg.Slide_bot.getCurrent(CurrentUnit.AMPS));

                telemetry.addData("arm rot value", -rbg.Arm_right.getCurrentPosition());
                telemetry.update();
                while(opModeIsActive()&&!gamepad1.share)
                {
                    rbg.delay(25);
                }

                rbg.pidfsetting(1839, rbg.pidf_hang2);
                rbg.delay(500);
                rbg.linearslideTq(6700,0.95);

                while(opModeIsActive() && rbg.Slide_top.getCurrentPosition() < 6600){
                    rbg.delay(25);
                }
                rbg.pidfsetting(2700, rbg.pidf_hang2);
                rbg.delay(1000);
                rbg.linearslideTq(6000,0.95);
                rbg.delay(500); // 2000
                rbg.pidfsetting(1576, rbg.pidf_hang2);
                rbg.delay(500);
                rbg.linearslideTq(-500,0.95);

                while(opModeIsActive() && !gamepad1.share){
                    rbg.delay(25);
                }
                rbg.linearslideTq(-500,0);
                rbg.delay(100000);
            }
           // if (gamepad2.ps) rbg.flag[rbg.force] = true;

          //  if (gamepad2.dpad_up && gamepad2.dpad_down)   rbg.adjust(gamepad2.dpad_up, gamepad2.dpad_down);
            rbg.robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, speed_factor);


//            telemetry.addData("TempInput", rbg.tempinput);
//            telemetry.update();

          telemetry.addData("armlinerslide", rbg.Slide_top.getCurrentPosition());
            telemetry.addData("armrotate position", -rbg.Arm_right.getCurrentPosition());


            telemetry.addData("Colors red", rbg.Intake_color.red());
            telemetry.addData("Colors green", rbg.Intake_color.green());
            telemetry.addData("Colors blue", rbg.Intake_color.blue());


            telemetry.addData("Intake HANDLE", rbg.  Intake_handle.getPosition());
            telemetry.addData("Intake rot", rbg.Intake_rot.getPosition());
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











