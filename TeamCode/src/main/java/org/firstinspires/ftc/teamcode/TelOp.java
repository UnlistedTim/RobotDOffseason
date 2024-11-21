package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
                        state=State.LIFT;
                        break;
                    }
                    if(!rbg.flag[rbg.button_flip]&&rbg.flag[rbg.intake_ready]&&!gamepad2.right_bumper)
                    {
                        rbg.flag[rbg.button_flip]=true;// prevent gunner hold the right bumper.
                    }
                    if(!rbg.flag[rbg.intake_ready] &&gamepad2.right_bumper){
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
            if (gamepad1.share) rbg.hang();
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

            if (gamepad2.ps) rbg.flag[rbg.force] = true;
            rbg.armrotatePIDF();
            if (gamepad2.dpad_up && gamepad2.dpad_down)   rbg.adjust(gamepad2.dpad_up, gamepad2.dpad_down);
            rbg.robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, speed_factor);


//            telemetry.addData("TempInput", rbg.tempinput);
//            telemetry.update();

//            if (rbg.driving) {
//                if (robo_drive)
//                    rbg.robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
//                else
//                    rbg.field_centric(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
//            }

//            telemetry.addData("flag",rbg.flag[rbg.specimen]);
//                telemetry.addData("slides target", rbg.armSlide.getTargetPosition());
//                telemetry.addData("slides real ", rbg.armSlide.getCurrentPosition());
//                telemetry.addData("rotate target", rbg.rotateTarget);
//                telemetry.addData("rotate real", rbg.armRotate.getCurrentPosition());
//                telemetry.update();
            }
        }
    }











