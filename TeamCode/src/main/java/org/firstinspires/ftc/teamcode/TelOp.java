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
                    if (gamepad2.left_bumper || rbg.flag[rbg.pre_spec]) {
                        if(rbg.pre_spec()){
                            speed_factor = 0.5;
                            state = State.SPECIMENINTAKE;
                            break;
                        }

                    }
                    if (gamepad2.right_bumper && rbg.flag[rbg.intake_ready] && rbg.flag[rbg.button_flip]) {
                        rbg.intake();
                        if (rbg.flag[rbg.color_check]) {
                            state = State.LIFT;
                            speed_factor = 1.0;
                            break;
                        }

                    }
                    if (!rbg.flag[rbg.button_flip] && rbg.flag[rbg.intake_ready] && !gamepad2.right_bumper) {
                        rbg.flag[rbg.button_flip] = true;// prevent gunner hold the right bumper.
                    }
                    if (!rbg.flag[rbg.intake_ready] && gamepad2.right_bumper) {
                        //rbg.pre_intake();
                        rbg.pre_intake();
                        speed_factor = 0.4;

                    }

                    if (rbg.flag[rbg.intake_ready]) {
                        rbg.intake_shift(gamepad2.right_stick_y, false);
                        rbg.pre_intake_adjust(gamepad2.left_stick_x, gamepad2.left_stick_y);
                    }

                    break;

                case SPECIMENINTAKE:
                    if (gamepad2.right_bumper) {
                        state = State.INTAKE;
                        rbg.pre_intake();
                        speed_factor = 0.4;
                        break;
                    }

                    if (gamepad2.left_bumper) {
                        rbg.pre_specimen();
                    }

                    if (gamepad1.right_bumper || rbg.flag[rbg.spec]) {
                        speed_factor = 1.0;
                        if (rbg.intake_specimen()) {
                            state = State.SPECIMENOUTTAKE;
                        }
                    }

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
                        speed_factor = 0.4;
                        break;
                    }

                    if (gamepad1.touchpad) {
                        rbg.intake_throw(); // throw sample, auto go to specimen pre intake
                        rbg.flag[rbg.intake_shift] = false;
                        speed_factor = 0.5;
                        state = State.SPECIMENINTAKE;
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

                case SPECIMENOUTTAKE:
                    if (gamepad1.right_bumper || rbg.flag[rbg.outtake]) {

                        if (rbg.outtake_spec()) {
                            speed_factor = 1.0;
                            state = State.INTAKE;
                            break;
                        }
                    }
//                    if (gamepad1.left_bumper) {
//                        rbg.spec_drop();
//                        state = State.SPECIMENINTAKE;
//                        speed_factor = 0.5;
//                        break;
//                    }
                    break;

            }

            rbg.armrotatePIDF();

            if (rbg.timer(86000, rbg.start) || rbg.flag[rbg.force]) {
                if (gamepad2.share || rbg.flag[rbg.hang]) {
                    rbg.pre_hang();
                }
                if (gamepad1.share) {
                    rbg.hang();
                }

            }


            if (gamepad2.ps) rbg.flag[rbg.force] = true;

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











