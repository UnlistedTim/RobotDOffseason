/*
Author: Jerome Yang
Start: 9/19/2024
End: #/##/####
Purpose: FTC 19571 The Robo Brigade Team B 24-25 season robot code
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Disabled
public class JeromeCode extends LinearOpMode {

    // time variables??? copied from tim... hee hee haa haa
    public ElapsedTime runtime = new ElapsedTime();
    boolean out_handling = false;
    double st = 0;// start time???

    // Status of Robot Actions
    boolean is_ScoreSpecimens = false;
    boolean is_Intake = false;
    boolean is_Intake_down = false;
    boolean is_Outtake = false;
    int sampleIntake_Position = 2;

    //arrays start at 0-infinite start with -1 b/c I want it to start at 1
    double[] array_arm    = new double[]{0, 0.42 , 0.42 , 0.42 , 0.425, 0.425, 0.425, 0.43 , 0.43 , 0.43 , 0.43 , 0.435, 0.435, 0.435, 0.435  };
    int[] array_rotate    = new int[]   {0, -100 , -82  , -66  , -52  , -38  , -26  , -14  , -4   , 0    , 6    , 10   , 14   , 16   ,  18    };
    double[] array_extend = new double[]{0, 1.26 , 1.22 , 1.18 , 1.14 , 1.10 , 1.06 , 1.02 , 1.02 , 1.02 , 1.01 , 1.01 , 1.01 , 1.01 , 1.01   };

    // extend linear slide variable
    int extendSlide_target_position = 0;
    int extendSlide_rest_position = 0;
    int extend_intakeSample = 800 - 465;// 800 - 465 (intake interval) b/c start at position 1
    int extendSlide_intakeSample_Interval = 465;
    int extend_intakeSpecimens = 0;
    int extend_sampleLow = 5000;
    int extend_sampleHigh = 10500;
    int extend_specimenLow = 1100;
    int extend_specimenLow_down = extend_sampleLow - 1000;
    int extend_specimenHigh = 3500;
    int extend_specimenHigh_down = extend_specimenHigh - 1700;

    int extendSlide_target_speed = 0;


    // rotate linear slide variable
    int rotateSlide_target_position = 0;
    int rotateSlide_rest_position = 100;
    int rotate_intakeSample = 0;
    int rotate_intakeSpecimens = 200;
    int rotate_sampleLow = 900;
    int rotate_sampleHigh = 900;
    int rotate_specimenLow = 500;
    int rotate_specimenHigh = 850;

    int rotateSlide_target_speed = 0;

    // grab servo variable
    double grab_rotate_target = 0.45;
    double grab_open = 0.38;
    double grab_close = 0.20;

    // hand servo variable
    double hand_rotate_target = 0.52;
    double hand_perpendicular = 0.52;
    //45 right = 0.34
    //45 left = 0.70
    double hand_parallel = 0.16;

    // arm servo variable
    double arm_rotate_target = 0.4;
    double arm_down = 0.4;
    double arm_lowered_SpecimenIntake = 0.68;
    double arm_forward = 0.74;
    double arm_sampleLow = 0.3;
    double arm_sampleHigh = 0.16;

    // motor and servo definitions
    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;

    DcMotorEx rotateSlideMotor;
    DcMotorEx extendSlideMotor;

    CRServo intake_servo;
    private ColorSensor intake_color;
    Servo handServo;
    Servo armServo;

    @Override

    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure ID's match the configuration
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        rotateSlideMotor = hardwareMap.get(DcMotorEx.class, "rotateSlideMotor");
        extendSlideMotor = hardwareMap.get(DcMotorEx.class, "extendSlideMotor");

        intake_servo = hardwareMap.get(CRServo.class, "intake_servo");
        intake_color = hardwareMap.get(ColorSensor.class,"intake_color");
        handServo = hardwareMap.get(Servo.class, "handServo");
        armServo = hardwareMap.get(Servo.class, "armServo");

        // reverse because it the only one spinning in wrong direction idk
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // reverse because of gears or something
        extendSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset the linear slide
        rotateSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotateSlideMotor.setTargetPosition(0);
        rotateSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotateSlideMotor.setVelocity(0);

        extendSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendSlideMotor.setTargetPosition(0);
        extendSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendSlideMotor.setVelocity(0);

        // make the motors stop if zero power
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set positions of motors/servos and move it.
        extendSlide_target_position = extendSlide_rest_position; extendSlide_target_speed = 0;
        rotateSlide_target_position = rotateSlide_rest_position; rotateSlide_target_speed = 500;

        sleep(500);

        grab_rotate_target = grab_open;
        hand_rotate_target = hand_perpendicular;
        arm_rotate_target = arm_down;

        extendSlideMotor.setVelocity(extendSlide_target_speed);
        extendSlideMotor.setTargetPosition(extendSlide_target_position);

        rotateSlideMotor.setVelocity(rotateSlide_target_speed);
        rotateSlideMotor.setTargetPosition(rotateSlide_target_position);

        sleep(100);
//
//        handServo.setPosition(hand_rotate_target);
//        armServo.setPosition(arm_rotate_target);

        // Before is all initialized
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.touchpad){
                intakeTest();
            }

            // drive the robot
            controllerDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, -gamepad1.left_stick_x);

            // change scoring place
            if (gamepad1.left_bumper || gamepad1.right_bumper) {
                changeScorePlace(gamepad1.left_bumper, gamepad1.right_bumper);
            }

            // Intake
            if (gamepad2.y || gamepad2.a){

                if (!is_Intake){

                    if (is_ScoreSpecimens) {
                        intakeOuttake(grab_open, hand_perpendicular, arm_lowered_SpecimenIntake, 100);
                        if (is_Outtake) {
                            move_extendSlide(extend_intakeSpecimens, 3000);
                            if(!out_handling) {
                                move_extendSlide(extend_intakeSpecimens, 3000);
                                timer(0);
                                out_handling = true;
                            }// not out handling end
                            if(timer(500) && out_handling){
                                out_handling = false;
                                moveSlide(rotate_intakeSpecimens, extend_intakeSpecimens, 100);
                            }// if timer end
                        }// is outtake end
                    }// is score specimens end

                    else if (!is_ScoreSpecimens) {
                        intakeOuttake(grab_open, hand_perpendicular, arm_down, 100);
                        if (is_Outtake) {
                            move_extendSlide(extend_intakeSample, 3000);
                            if(!out_handling) {
                                move_extendSlide(extend_intakeSample, 3000);
                                timer(0);
                                out_handling = true;
                            }// not out handling end
                            if(timer(1000) && out_handling){
                                out_handling = false;
                                moveSlide(rotateSlide_rest_position, extend_intakeSample + extendSlide_intakeSample_Interval, 700);
                                moveSlide(rotate_intakeSample, extend_intakeSample + extendSlide_intakeSample_Interval, 300);
                            }// if timer end
                        }// is outtake end
                    }// if not score specimen end
                    is_Intake_down = false;

                }// not is intake end

                else if (is_Intake){
                    if (!is_ScoreSpecimens) {
                        if (gamepad2.y) {
                            intakeOuttake(grab_close, hand_perpendicular, arm_down, 200);
                            moveSlide(rotateSlide_rest_position, extend_intakeSample + extendSlide_intakeSample_Interval * sampleIntake_Position, 200);
                            is_Intake_down = false;
                        }
                        else if (gamepad2.a) {
                            is_Intake_down = true;
                        }
                    }
                }// is intake end

                is_Intake = true;
                is_Outtake = false;
            }// intake end

            // adjust intake
            if (is_Intake && !is_ScoreSpecimens) {
                adjustIntake(gamepad2.left_stick_x, gamepad2.right_stick_y, sampleIntake_Position);
            }

            // Outtake and change position of scoring
            if (gamepad2.dpad_up || gamepad2.dpad_down) {
                if (is_Intake){
                    if (is_ScoreSpecimens) {
                        if (gamepad2.dpad_up) {
                            // reversed (servos, then motors) because of wall
                            moveSlide(rotate_specimenHigh, extend_specimenHigh, 100); // 100 sleepTime for specimens to allow driver to move robot earlier
                            intakeOuttake(grab_close, hand_perpendicular, arm_down, 100);
                        }
                        else if (gamepad2.dpad_down){
                            moveSlide(rotate_specimenLow, extend_specimenLow, 100);
                            intakeOuttake(grab_close, hand_perpendicular, arm_down, 100);
                        }
                    }// is score specimens end
                    else if (!is_ScoreSpecimens){
                        if (gamepad2.dpad_up) {
                            intakeOuttake(grab_close, hand_parallel, arm_sampleHigh, 200);
                            moveSlide(rotate_sampleHigh, extend_sampleHigh, 200);
                        }
                        else if (gamepad2.dpad_down){
                            intakeOuttake(grab_close, hand_parallel, arm_sampleHigh, 200);
                            moveSlide(rotate_sampleLow, extend_sampleLow, 200);
                        }
                    }// not is score specimens end
                }// is intake end
                else if (!is_Intake) {
                    changeScoringPosition();
                }
                is_Intake = false;
                is_Intake_down = false;
                is_Outtake = true;
            }// outtake end

            // pick up and score samples
            if (gamepad2.right_bumper){
                grabScore();
            }

            if (gamepad2.left_bumper){
                specimenScore();
            }

            // hang
            /*
            if (gamepad1.dpad_up){
                hang(motor_SleepTime);
            }
             */

            // update telemetry or like what you see
            updateTelementry();

        }// op mode active end

    } // run OpMode end

    // ALL THE METHODS FUNCTIONS WHATEVER BELOW

    // FUNCTIONS THAT ARE IN OTHER FUNCTIONS/ HELPER FUNCTIONS I THINK!!!
    public void stopDrive(){
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public void move_extendSlide(int position, int speed){
        extendSlide_target_position = position;
        extendSlide_target_speed = speed;
        extendSlideMotor.setVelocity(extendSlide_target_speed);
        extendSlideMotor.setTargetPosition(extendSlide_target_position);
    }
    public void move_rotateSlide(int position, int speed){
        rotateSlide_target_position = position;
        rotateSlide_target_speed = speed;
        rotateSlideMotor.setVelocity(rotateSlide_target_speed);
        rotateSlideMotor.setTargetPosition(rotateSlide_target_position);
    }
    public void move_grabServo(double target){
        grab_rotate_target = target;
        //grabServo.setPosition(grab_rotate_target);
    }
    public void move_handServo(double target){
        hand_rotate_target = target;
        handServo.setPosition(hand_rotate_target);
    }
    public void move_armServo(double target){
        arm_rotate_target = target;
        armServo.setPosition(arm_rotate_target);
    }
    // FUNCTIONS THAT MOVE ROBOT/ ARE IN MAIN CODE!!!
    boolean timer(double period) {
        // used to reset the start time?
        if (period == 0) {
            st = runtime.milliseconds();
            return false;
        }
        if (runtime.milliseconds() - st > period) return true;
        return false;
    }//timer end
    public void controllerDrive(double rightStickX, double rightStickY, double leftStickX){
        double x = rightStickX * 1.1; // Counteract imperfect strafing
        double y = -rightStickY; // Remember, Y stick value is reversed
        double rx = leftStickX * 0.7; // turning speed

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }// controller drive end
    public void changeScorePlace(boolean leftBumper, boolean rightBumper){
        if (rightBumper){
            is_ScoreSpecimens = true;
        }
        else if (leftBumper){
            is_ScoreSpecimens = false;
        }
    }// changeScorePlace end
    public void intakeOuttake(double grab, double hand, double arm, int sleepTime){

        grab_rotate_target = grab;
        hand_rotate_target = hand;
        arm_rotate_target = arm;

        //grabServo.setPosition(grab_rotate_target);
        sleep(sleepTime);
        handServo.setPosition(hand_rotate_target);
        sleep(sleepTime);
        armServo.setPosition(arm_rotate_target);
        sleep(sleepTime);
    }// intake outtake end
    public void moveSlide(int rotateSlide, int extendSlide, int sleepTime){

        rotateSlide_target_speed = 2000;
        rotateSlide_target_position = rotateSlide;

        extendSlide_target_speed = 3000;
        extendSlide_target_position = extendSlide;

        rotateSlideMotor.setVelocity(rotateSlide_target_speed);
        rotateSlideMotor.setTargetPosition(rotateSlide_target_position);

        sleep(sleepTime);

        extendSlideMotor.setVelocity(extendSlide_target_speed);
        extendSlideMotor.setTargetPosition(extendSlide_target_position);

        sleep(sleepTime);
    }// moveSlide end
    public void adjustIntake(double leftX, double rightY, int position){
        if (rightY < -0.1){
            sampleIntake_Position += 1;
        }
        else if (rightY > 0.1){
            sampleIntake_Position -= 1;
        }
        if (sampleIntake_Position < 2){
            sampleIntake_Position = 2;
        }
        if (sampleIntake_Position > 7){
            sampleIntake_Position = 7;
        }
        move_handServo(hand_rotate_target += (leftX * 0.15) );
        sleep(150);
        if (!is_Intake_down){
            move_extendSlide(extendSlide_target_position = extend_intakeSample + (sampleIntake_Position * extendSlide_intakeSample_Interval) , 2500);
        }
        else if (is_Intake_down) {
            intakeOuttake(grab_rotate_target, hand_rotate_target, array_arm[position], 0);
            moveSlide(array_rotate[position], (int) ((extend_intakeSample + extendSlide_intakeSample_Interval * position)  * array_extend[position]) , 0);
        }// if is intake down end
    }// adjust intake end
    public void changeScoringPosition(){
        if (gamepad2.dpad_up) {
            if (is_ScoreSpecimens) {
                rotateSlide_target_position = rotate_specimenHigh;
                extendSlide_target_position = extend_specimenHigh;

                grab_rotate_target = grab_close;
                hand_rotate_target = hand_perpendicular;
                arm_rotate_target = arm_down;
            }
            else{
                rotateSlide_target_position = rotate_sampleHigh;
                extendSlide_target_position = extend_sampleHigh;

                grab_rotate_target = grab_close;
                hand_rotate_target = hand_parallel;
                arm_rotate_target = arm_sampleHigh;
            }
        }
        else if (gamepad2.dpad_down){
            if (is_ScoreSpecimens) {
                rotateSlide_target_position = rotate_specimenLow;
                extendSlide_target_position = extend_specimenLow;

                grab_rotate_target = grab_close;
                hand_rotate_target = hand_perpendicular;
                arm_rotate_target = arm_down;
            }
            else{
                rotateSlide_target_position = rotate_sampleLow;
                extendSlide_target_position = extend_sampleLow;

                grab_rotate_target = grab_close;
                hand_rotate_target = hand_parallel;
                arm_rotate_target = arm_sampleLow;
            }
        }
        rotateSlideMotor.setVelocity(rotateSlide_target_speed);
        rotateSlideMotor.setTargetPosition(rotateSlide_target_position);

        extendSlideMotor.setVelocity(extendSlide_target_speed);
        extendSlideMotor.setTargetPosition(extendSlide_target_position);

        //grabServo.setPosition(grab_rotate_target);
        handServo.setPosition(hand_rotate_target);
        armServo.setPosition(arm_rotate_target);
    }// change scoring position end
    public void grabScore(){
        if (grab_rotate_target == grab_open){
            grab_rotate_target = grab_close;
        }
        else if (grab_rotate_target == grab_close){
            grab_rotate_target = grab_open;
        }
        //grabServo.setPosition(grab_rotate_target);
        sleep(200);
    }// grabScore end
    public void intakeTest(){

        intake_servo.setPower(-0.6);

        boolean red = false;
        boolean yellow = false;
        boolean blue = false;

        while(opModeIsActive() && !(blue && red)){
            blue = intake_color.blue() > 200;
            red = intake_color.red() < 100;

            telemetry.addData("Colors red", intake_color.red());
            telemetry.addData("Colors green", intake_color.green());
            telemetry.addData("Colors blue", intake_color.blue());
            telemetry.update();


        }
        intake_servo.setPower(0);

        if (blue){
            telemetry.addData("Color detected blue", intake_color.blue() );
        }
        else if (red){
            telemetry.addData("Color detected red", intake_color.red() );
        }

        telemetry.update();





        timer(5000);

    }
    public void specimenScore(){
        if (is_Outtake) {
            if (extendSlide_target_position == extend_specimenHigh) {
                extendSlide_target_position = extend_specimenHigh_down;
            } else if (extendSlide_target_position == extend_specimenHigh_down) {
                extendSlide_target_position = extend_specimenHigh;
            }
            else if (extendSlide_target_position == extend_specimenLow){
                extendSlide_target_position = extend_specimenLow_down;
            }
            else if (extendSlide_target_position == extend_specimenLow_down){
                extendSlide_target_position = extend_specimenLow;
            }
        }
        move_extendSlide(extendSlide_target_position, 1500);

        sleep(250);
    }// specimen score end
    public void hang(int sleepTime){
        //steps in team b discord
        //move servos so not in the way
        move_grabServo(grab_close);
        move_handServo(hand_parallel);
        move_armServo(arm_forward);

        //1st stage 1st step: rotate the arm minus and point low rung
        move_rotateSlide(-500,500);
        sleep(sleepTime);
        //2nd: extend the motor plus
        move_extendSlide(500,500);
        sleep(sleepTime);
        //3rd
        move_rotateSlide(-300,500);
        sleep(sleepTime);
        //4th
        move_rotateSlide(-100,500);
        sleep(sleepTime);
        //5th
        move_extendSlide(50,500);
        sleep(sleepTime);
        //6th
        move_rotateSlide(0,500);
        sleep(sleepTime);
        //2nd stage 7th
        move_extendSlide(500,500);
        sleep(sleepTime);
        //8th
        move_rotateSlide(-100,500);
        sleep(sleepTime);
        //9th
        move_extendSlide(0,500);
        sleep(sleepTime);
        //10th
        move_rotateSlide(-100,500);
        sleep(sleepTime);
        //11th
        move_extendSlide(0,500);
        sleep(sleepTime);
        //12th
        move_rotateSlide(0,500);
        sleep(sleepTime);
    }// hang end
    public void updateTelementry(){
//        telemetry.addData("Elapsed time: ", runtime);
//        telemetry.addData("out_Handling: ", out_handling);
//        telemetry.addData("start time: ", st);
//        telemetry.addLine();
//        telemetry.addData("isScoreSpecimens", is_ScoreSpecimens);
//        telemetry.addData("isIntake", is_Intake);
//        telemetry.addData("isOuttake", is_Outtake);
//        telemetry.addData("Intake Position", sampleIntake_Position);
//        telemetry.addLine();
//        telemetry.addData("Target Length of Linear slide:", extendSlide_target_position);
//        telemetry.addData("Length of linear slide:", extendSlideMotor.getCurrentPosition());
//        telemetry.addLine();
//        telemetry.addData("Target Rotation of Linear slide:", rotateSlide_target_position);
//        telemetry.addData("Rotation of linear slide:", rotateSlideMotor.getCurrentPosition());
//        telemetry.addLine();
//        telemetry.addData("Target of Arm Servo:", arm_rotate_target);
//        telemetry.addData("Arm Servo:", armServo.getPosition());
//        telemetry.addLine();
//        telemetry.addData("Target of Hand Servo:", hand_rotate_target);
//        telemetry.addData("Hand Servo:", handServo.getPosition());
//        telemetry.addLine();
//        telemetry.addData("Target of Grab Servo:", grab_rotate_target);
//        //telemetry.addData("Grab Servo:", grabServo.getPosition());
//        telemetry.addLine();
//        telemetry.addData("1 Left Joystick X", gamepad1.left_stick_x);
//        telemetry.addData("1 Left Joystick Y", gamepad1.left_stick_y);
//        telemetry.addLine();
//        telemetry.addData("1 Right Joystick X", gamepad1.right_stick_x);
//        telemetry.addData("1 Right Joystick Y", gamepad1.right_stick_y);
//        telemetry.addLine();
//        telemetry.addData("2 Left Joystick X", gamepad2.left_stick_x);
//        telemetry.addData("2 Left Joystick Y", gamepad2.left_stick_y);
//        telemetry.addLine();
//        telemetry.addData("2 Right Joystick X", gamepad2.right_stick_x);
//        telemetry.addData("2 Right Joystick Y", gamepad2.right_stick_y);
//        telemetry.addLine();
//        telemetry.addData("2 Left Trigger ", gamepad2.left_trigger);
//        telemetry.addData("2 Right Trigger ", gamepad2.right_trigger);
//        telemetry.addLine();
//        telemetry.update();
    }//update telementry end

} // class end