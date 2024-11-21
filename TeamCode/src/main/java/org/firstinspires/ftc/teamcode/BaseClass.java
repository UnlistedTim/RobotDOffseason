package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.controller.PIDController;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import   java.lang.Math;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Action;


public class BaseClass extends MecanumDrive {

    // Ticks from old rpm to new rpm for slide factor 0.4769124742


    public ElapsedTime runtime = new ElapsedTime();
    public LinearOpMode Op;


   // int armslideV1 = 1700, armslideV2 = 2300, armslideV3 = 2750;//max 2800





   // int armslideOutSpec = 858 /*1800*/;

    double intakerotpose=0,gearboxpose=0,gearboxcurrent=0;
    double arm_rot_power = 0;
    int arm_slide_pos = 0;
    int lslo=0,lshi=2351;

    int armslidePreSpec = 0;



    double rotateStartangle=-47 ;

    int rotateTarget=0,crotateTarget=0;





    int armrotateHang1 = 2800,armrotateHang2 = 3500 ,armrotateHang3 = 3900, armrotateHang4 = 325;


    PIDController controller;
    final double ticks_in_degree = 8192/360;
    int rotatePos,slidePos,absrotategap;


    double p = 0.00004, i = 0, d = 0.0001 ,f = 0.12,k=0.00003;//k = 0.00001;
// idle, specman slides=400,rotate 380-,


    double handlePos = 0.05, handleH, handleL, handleStep = 0.05, handleCurrent = 0.05;
    double handleIn1 = 0.44, handleIn2 = 0.475;
    double handleOut1 = 0.21, handleIdle = 0.33;//0.33 0.21
    double handleInSpec = 0.14,handlesina=0.16;
    double handleHang = 0.1;
    double handleOutSpec = 0.235, handleOutSpeca=0,handlepOutSpeca=0.1;

    double clawPos = 0.3, clawStep = 0.05, clawCurrent = 0.3, clawH, clawL;//0.24

    public static boolean baseblue = false, baseright = true;

    double[][] moveconfig= new double[20][20];
    int speedg=0,strafeg=1,turng=2,speedmax=3,strafemax=4,turnmax=5,xdis=6,ydis=7,adis=8,time=9;
    double[][] pidftable= new double[20][3];
    int pidf_intake_up=0,pidf_intake_down=1, pidf_outtake_down=2,pidf_outtake_up=3;
    int pp=0,ii=1,dd=2;


// 3rd robot special

    int roatate_prein0=60, rotate_in0=-50, roatate_prein1=80,rotate_in1=-30, roatate_prein2=80,rotate_in2=-30,rotate_idle=-120,rotate_outtake=2350;//intake rotat could not over 70
    int slide_in0=800,slide_in1=1200,slide_in2=1800,slide_idle=800,slide_outtake=2500,slide_rotate=1300;
    int intake_level=0;
    double handle_idle=0.05,handle_intake=0.36,handl_specimen_intake=0.05,handle_outtake=0;
    double handlerot_intake=0.55; // 0,0.3.0.75,1;
    int   intake_levelnumber=30;


    int [] rotate= new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int [] slide= new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    int slidev0=1000,slidev1=1800,slidev2=2500;
 //   double[] handle = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int idle=0,pin1=1,in1=2,pin2=3,in2=4,sin=5,sout=6,hang1=7,hang2=8,hang3=9,out=10,psout = 11,souta=12,psouta=13,sina=14;




    boolean[] flag = new boolean[]{false,false,false,false, false, false, false, false,false, false, false, false, false, false, false, false, false,false, false, false, false, false, false,false};
    double[] stoptime = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0};
    int[] step = new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0};
    final int start = 0, first = 1, intake = 2, lift = 3, outtake = 4, intake_shift = 5, button_flip = 6;
    final int intake_adjustment = 7, specimenfirst= 8, adjust = 9, drive = 10, intake_ready = 11, hang = 12, specimen = 13,pause=14;
    final int force = 15,delay=16, hang0 = 17,spicemen0=18,debug=19,asin=20,push=21;
    double soutadis=260 , sinadis=150; //250









    final double RPS = 2787 * 0.9;




    public BaseClass(LinearOpMode linearOpMode, Pose2d pose) {
        super(linearOpMode.hardwareMap, pose);
        Op = linearOpMode;

    }

    public BaseClass(HardwareMap hardwareMap, Pose2d pose1) {
        super(hardwareMap, pose1);

    }
//    public class ARM()
//    {
//        DcMotorEx arm1,arm2;
//       void( ARM(DcMotorEx motor1, DcMotorEx m)
//        {
//
//
//    }
//
//        void setpower(){
//
//
//        }
//
//
//    }


    public void strafe(double power) {
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(power);
    }


    public void straft(boolean right, double power) {

        double dir = 1;
        if (right) dir = -1;
        // double  power=1;
        leftFront.setPower(dir * (power + 0.035));
        leftBack.setPower(-dir * power);
        rightFront.setPower(-dir * (power + 0.03));
        rightBack.setPower(dir * power);
    }

    protected void robot_centric(double iy, double ix, double irx, double ratio) {
        if (!flag[drive]) return;
        double y = -iy;
        double x = ix * 1.1; // Counteract imperfect strafing
        double rx = irx * 0.7; // 0.75
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower * ratio);
        rightFront.setPower(frontRightPower * ratio);
        leftBack.setPower(backLeftPower * ratio);
        rightBack.setPower(backRightPower * ratio);


    }





//    public void reset(){
//
//        armSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armSlide.setPower(-0.8);
//        pause(800);
//        armSlide.setPower(-0.3);
//        pause(500);
//        armSlide.setPower(0);
//
//        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armSlide.setTargetPosition(0);
//        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armSlide.setVelocity(0);
//        pause(500);
//
//
//        armRotate.setPower(-0.5);
//        armRotateLeft.setPower(0.5);
//        pause(1700);
//        armRotate.setPower(-0.1);
//        armRotateLeft.setPower(0.1);
//        pause(500);
//        armRotate.setPower(0);
//        armRotateLeft.setPower(0);
//
//        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armRotateLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armRotateLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armRotate.setPower(0);
//        armRotateLeft.setPower(0);
//        pause(300);
//        return;
//
//
//
//
//    }




    public void hang() {
        if (!timer(86000, start) && (!flag[force])) return;

        stop_drive();
     //   linearslide( 0, armslideV3);


        move(-0.25);
        Intake_handle.setPosition(handleHang);

        flag[hang0] = true;

        rotatetargetPIDF(armrotateHang1);
        armrotatePIDF();

        flag[hang0] = false;


        timer(0,hang);
        while((Arm_left.getCurrentPosition() < (armrotateHang1 - 100) || Slide_top.getCurrentPosition() > 300) && Op.opModeIsActive() && !timer(2000,hang)){

            pause(20);
            armrotatePIDF();
        }
        delay(300);


        rotatetargetPIDF(armrotateHang2);

        armrotatePIDF();

        timer(0,hang);



        while(  Arm_left.getCurrentPosition() < (armrotateHang2-80) && Op.opModeIsActive() && !timer(1000,hang)){
            armrotatePIDF();
            pause(15);
        }

        stop_drive();
        rotatetargetPIDF(armrotateHang3);
        armrotatePIDF();


        delay(300);

        move(0.25);
        delay(300);

        timer(0,hang);

        while(Arm_left.getCurrentPosition() < (armrotateHang3 - 200) && Op.opModeIsActive() && !timer(800,hang)){
            armrotatePIDF();
            pause(15);
        }
        pause(300);
        flag[hang] = true;
        rotatetargetPIDF(armrotateHang4);
        armrotatePIDF();
        delay(1000);
        stop_drive();

        delay(300000);












//        pause(2000); // delay time
//        stop_drive();
//        linearslide(armRotate, 2310, armrotateV0);
//        pause(800);
//        stop_drive();
//        flag[hang] = true;
//
//        // rot to here
//        move(0.18); // transitions to forward moving, still turning arm more same direction
//        pause(800); // delay time
//        linearslide(armRotate, 150, armrotateV1); // Turn opposite direction to hang on
//        pause(5000);// end;
//        stop_drive();
//        pause(300000);

    }




    //hang
//    public void hang() {
//        stop_drive();
//
//        if (!timer(87000, start) && (!flag[force])) return;
//        linearslide(armSlide, slide[hang1], armslideV3);
//        Handle.setPosition(handleHang);
//      //  linearslide(armRotate, 1725, armrotateV1); // arm rotate to get ready for hang
//        rotatetargetPIDF(armrotateHang1);
//        armrotatePIDF();
//     //   delay(800);
//        move(-0.15);
//       // delay(2000); // delay time
//      //  stop_drive();
//        while(armSlide.getCurrentPosition()>100)
//        {
//            pause(20);
//            armrotatePIDF();
//        }
//        rotatetargetPIDF(armrotateHang2);
//        stop_drive();
//        //linearslide(armRotate, 2310, armrotateV0);
//       // delay(800);
//        delay(1000);
//        // rot to here
//        move(0.2); // transitions to forward moving, still turning arm more same direction
//        delay(800); // delay time
//      //  linearslide(armRotate, 150, armrotateV1); // Turn opposite direction to hang on
//        rotatetargetPIDF(armrotateHang3);
//        delay(5000);// end;
//        stop_drive();
//        delay(300000);
//    }
    public void aspicmenintake() {
        double dis = 500;
        stop_drive();
      //  rotatetargetPIDF(rotate[sina]);
        Intake_handle.setPosition(handlesina);
       // delay(400);
        if(!flag[push]){
            move(0.4);
        delay(250);
        }
        else {
            flag[push] = false;
            move(0.4);
            delay (160);
        }

        move(0.17);
        timer(0,5);
        while (Op.opModeIsActive() && (dis > sinadis) &&!timer(900,5)) {
            armrotatePIDF();
           // dis=frontdis.getDistance(DistanceUnit.MM);
        }
        stop_drive();
        move(-0.45);

        delay(50);
//        Claw.setPosition(clawClose);
        stop_drive();
       // delay(1500);
       // Claw.setPosition(clawClose);
        delay(300);
        rotatetargetPIDF(rotate[in2]);
      //  Handle.setPosition(handleOutSpeca);
        delay(150);
        move(-0.6);
        delay(350);
        stop_drive();
        //delay(100);
    }



//    public void intake_shift(boolean drop) {
//        if (!timer(500, intake_shift)) return;
//
//        flag[specimen] = false;
////        Claw.setPosition(clawIntake);
//        if (!drop) {
//            flag[intake_shift] = !flag[intake_shift];// intake 1
//            timer(0, intake_shift);
//        }
//
//        if (flag[intake_shift]) {
//            linearslide( slide[pin1], armslideV3);
//          //  linearslide(armRotate, armrotatePrein1, armrotateV1);
//            rotatetargetPIDF(rotate[pin1]);
//            Intake_handle.setPosition(handleIn1);
//        } else {
//           // linearslide(armRotate, armrotatePrein2, armrotateV2);
//            rotatetargetPIDF(rotate[pin2]);
//            linearslide( slide[pin2], armslideV3);
//            Intake_handle.setPosition(handleIn2);
//        }
//        flag[intake_ready] = true;
//
//    }


    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFront.setPower(leftFrontPower);
       rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
       rightBack.setPower(rightBackPower);
    }

    public void intake_drop() {

        flag[lift] = false;

        Intake.setPower(0.9);
        delay(500);
        Intake_handle.setPosition(handleIdle);
        if (timer(4000, intake)) {
           idle();
        } else
        {
            Intake_handle.setPosition(handle_intake);
            Intake_rot.setPosition(handlerot_intake);
            pidfsetting(roatate_prein0, pidf_intake_up);
            intake_shift(0,true);
            flag[intake_ready]=true;
        }

    }

    public void adjust(boolean up, boolean down) {
        int target;
        //int pos = armRotate.getTargetPosition();
        // if (flag[offset] = true) return;
        if (!timer(250, adjust)) return;

        if (up) {
          target=rotateTarget+60;
          timer(0, adjust);
          rotatetargetPIDF( target);
            return;
        }
        if (down) {
            target =rotateTarget-60;
            timer(0, adjust);
            rotatetargetPIDF( target);
        }
     //   armRotate.setVelocity(armrotateV1);
//        if(left)  {linearslide(armSlide,armSlide.getTargetPosition()-50, armslideV1);timer(0,adjust);}
//        if(right)  {linearslide(armRotate,armRotate.getTargetPosition()+50, armslideV1); timer(0,adjust);}

    }




    public boolean timer(double period, int i) {

        if (period == 0) {
            stoptime[i] = runtime.milliseconds();
            return false;
        }
        return runtime.milliseconds() - stoptime[i] > period;
    }

    public void specimen_drop(){
        flag[lift] = false;
        flag[specimen] = true;

      //  linearslide(slide[idle],armslideV3);
        rotatetargetPIDF(rotate[idle]);
      //  linearslide(armRotate,armrotatePreinSpec,armrotateV3);
//        Claw.setPosition(clawOpen);
        Intake_handle.setPosition(handleInSpec);
        flag[intake_ready] = true;

    }

    public void intake_throw(){
        flag[lift] = false;
        Intake.setPower(0.9);
        delay(400);

    }


    // l
    public void specimen_pre(){
        flag[specimen] = true;
      //  linearslide(slide[sin],armslideV3);
     //   linearslide(armRotate,armrotatePreinSpec,armrotateV3);
        rotatetargetPIDF(rotate[sin]);
//        Claw.setPosition(clawOpen);
        Intake_handle.setPosition(handleInSpec);
          flag[intake_ready] = true;
    }


    public void fl_vel(double percent) {
        if (percent > 1.0 || percent < -1.0) return;
        if (Math.abs(percent * RPS) < 50) {
            leftFront.setPower(0);
            return;
        }
        leftFront.setVelocity(percent * RPS);
    }

    public void fr_vel(double percent) {
        if (percent > 1.0 || percent < -1.0) return;
        if (Math.abs(percent * RPS) < 50) {
            rightFront.setPower(0);
            return;
        }
        rightFront.setVelocity(percent * RPS);
    }
    public void pre_intake_adjust(double adjustx, double adjusty) {
        if(!timer(300,intake_adjustment)) return;
        if((Math.abs(adjustx)<0.7&&Math.abs(adjusty)<0.7)) return;
        int sl=Slide_bot.getTargetPosition();
        double tar=Intake_rot.getPosition();
        timer(0,intake_adjustment);
        if(adjusty<-0.7 && sl<slide_in2)  {linearslide(sl+100,slidev2) ;return;}
        if(adjusty>0.7 && sl>slide_in0)  {linearslide(sl-100,slidev2) ;return;}
        if(adjustx<-0.7)  {Intake_rot.setPosition(tar-0.25);return;}
        if(adjustx>0.7)  {Intake_rot.setPosition(tar+0.25);;}


    }
    public void pre_intake()

    {
            Intake_handle.setPosition(handle_intake);
            Intake_rot.setPosition(handlerot_intake);
            linearslide(slide_in0, slidev0);
            pidfsetting(roatate_prein0, pidf_intake_up);
            flag[intake_ready]=true;
            intake_level=0;
            timer(0,intake);

    }

    public void intake_shift(double sticky,boolean drop)

    {
        if(!drop) {
            if (!timer(500, intake_shift) || Math.abs(gamepad2.right_stick_y) < 0.6) return;

            if (sticky < 0) intake_level++;
            else intake_level--;

            if (intake_level > 2) intake_level = 0;
            else if (intake_level < 0) intake_level = 2;
        }
        timer(intake_shift,0);


        if(intake_level==0) {
            linearslide(slide_in0, slidev0);
           // pidfsetting(roatate_prein0, pidf_intake_up);
            return;
        }
        if(intake_level==1){
            linearslide(slide_in1, slidev0);
          //  pidfsetting(roatate_prein1, pidf_intake_up);
            return;
        }

        if(intake_level==3){
            linearslide(slide_in2, slidev0);
          //  pidfsetting(roatate_prein2, pidf_intake_up);

        }

    }




        public void preintake(double sticky)

    {


        if(sticky<0) intake_level++ ;
        else intake_level--;

       // intake_level=intake_levelnumber%3;
        if(!flag[intake_ready]) {
            Intake_handle.setPosition(handle_intake);
            Intake_rot.setPosition(handlerot_intake);
            linearslide(slide_in0, slidev0);
            pidfsetting(roatate_prein0, pidf_intake_up);
            flag[intake_ready]=true;
            intake_level=0;
            return;
        }


        if(intake_level==0) {
            linearslide(slide_in0, slidev0);
            pidfsetting(roatate_prein0, pidf_intake_up);
            return;
        }
        if(intake_level==1){
            linearslide(slide_in1, slidev0);
            pidfsetting(roatate_prein1, pidf_intake_up);
            return;
        }

        if(intake_level==3){
            linearslide(slide_in2, slidev0);
            pidfsetting(roatate_prein2, pidf_intake_up);

        }

    }
    public void idle()

    {
        Intake_handle.setPosition(handle_idle);
        Intake_rot.setPosition(handlerot_intake);
        linearslide(slide_in0, slidev0);
        pidfsetting(rotate_idle, pidf_intake_up);

    }



    public void intake()

    {

        if(!timer(500,intake)) return;
        pidfsetting(rotate_in0,pidf_intake_down);
        delay(300);
        Intake.setPower(-0.9);
        delay(1000);
        Intake.setPower(0);
        Intake_handle.setPosition(handle_outtake);
        Intake_rot.setPosition(handlerot_intake);
        pidfsetting(rotate_idle,pidf_intake_up);
        linearslide(slide_idle,slidev2);
        flag[intake_ready]=false;
        flag[button_flip]=false;
       // intake_level=0;// might drop
        delay(500);

    }

    public void rl_vel(double percent) {
        if (percent > 1.0 || percent < -1.0) return;
        if (Math.abs(percent * RPS) < 50) {
            leftBack.setPower(0);
            return;
        }
        leftBack.setVelocity(percent * RPS);
    }

    public void rr_vel(double percent) {
        if (percent > 1.0 || percent < -1.0) return;
        if (Math.abs(percent * RPS) < 50) {
            rightBack.setPower(0);
            return;
        }
        rightBack.setVelocity(percent * RPS);
    }

    public void motor_vel(DcMotorEx mot, double percent, boolean ignore) {
        //if (percent > 1.0 || percent < -1.0) return;
        if (ignore && Math.abs(percent) < 0.05) percent = 0;
        mot.setVelocity(percent * RPS);
    }


    public void move(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

    public void stop_drive() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }


    public void linearslide( int target, int speed) {

        if (target > lshi || target < lslo) return;
        Slide_top.setTargetPosition(-target);
        Slide_bot.setTargetPosition(target);
        Slide_top.setVelocity(speed);
        Slide_bot.setVelocity(speed);

    }

    public void pidfsetting(int target, int index)

    {
        p=pidftable[index][pp];
        i=pidftable[index][ii];
        d=pidftable[index][dd];
        rotateTarget=target;
        armrotatePIDF();
    }



    public void rotatetargetPIDF(int target) {

        int gap;
        crotateTarget=  rotateTarget;
        rotateTarget=target;
        gap=rotateTarget-crotateTarget;
        i=0;

// up from 100 t0 450  d=0.00002; p=0.0020 down p=0.00001;
        //down from pin2 to in2 p=0.0028,k=0.00003, i=0.00001;

        if (flag[hang0]){
            p = 0.001;
            d = 0.00004 ;//k = 0.00001;
            return;
        }


        if (flag[hang]){
            p = 0.002;
            d = 0;
            return;
        }

        if(flag[spicemen0])
        {
         p=0.0025;
         d=0;
         return;
        }
        if(flag[first])
        {
            p=0.003; d=0.00002;
            flag[first]=false;
            return;
        }
        if(flag[specimenfirst]){
            p = 0.00045;//p=0004
            d = 0.00006;
            i=0.00005;
            flag[specimenfirst]=false;
            return;

        }
        if(gap>3100){
            p = 0.00051;//p=0004
            d = 0.00005;
            i=0.00005;
            return;

        }


        if(gap>2000){
              p = 0.0005;//p=0004
              d = 0.00003;
              return;

        }

        if(gap<-2000){
            p=0.0003;
            d=0.00002;
            return;

        }
        if(gap<-700){
            p=0.001;
            d=0.00002;
            return;

        }
        if(gap>700){
            p=0.0018;
            d=0.00007;
            return;

        }

        if (gap<0 && ( gap >-100) ){
            p= 0.0024;
            d=  0.00002  ;
            i=0.00002;
            return;
        }


        if(gap>0) {
            p=0.0024; d=0.00002;
            return;
        }

      //gap<=0;
             p=0.0010;d=0.00002;




    }


    public final void pause(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    void linearslides_reset(){
        Arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_left.setTargetPosition(0);
        Arm_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_left.setVelocity(0);

        Slide_top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide_top.setTargetPosition(0);
        Slide_top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide_top.setVelocity(0);
    }



    public void  moveRobot(double x, double y, double yaw, int intake, boolean auton) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }




        // Send powers to the wheels.
        leftFront.setPower(leftFrontPower * intake);
        leftBack.setPower(leftBackPower * intake);
        rightFront.setPower(rightFrontPower * intake);
        rightBack.setPower(rightBackPower * intake);

    }

    public void init(int tstep)//

    {
       if(tstep==0) { //only for Teleop

           controller = new PIDController(p, i, d);
           Gearbox.setPosition(0.05);
           Intake_rot.setPosition(handlerot_intake);
           Arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
           Arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           Arm_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           Arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
           Arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           Arm_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

           Slide_bot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
           Slide_bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           Slide_bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Slide_bot.setVelocity(0);
//        Slide_top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

           Slide_top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           Slide_top.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           Slide_top.setPower(0);
           Slide_bot.setPower(0);
           Arm_right.setPower(0);
           Arm_left.setPower(0);

           // flag[intake_shift] = false;
           flag[drive] = true;

           pidftable[pidf_intake_up][pp]=0.004;  pidftable[pidf_intake_up][ii]=0;  pidftable[pidf_intake_up][dd]=0.00008;
           pidftable[pidf_intake_down][pp]=0.0035;  pidftable[pidf_intake_down][ii]=0;  pidftable[pidf_intake_down][dd]=0.00005;
           pidftable[pidf_outtake_up][pp]=0.002;  pidftable[pidf_intake_down][ii]=0;  pidftable[pidf_intake_down][dd]=0.00005;
           pidftable[pidf_outtake_down][pp]=0.0015;  pidftable[pidf_intake_down][ii]=0;  pidftable[pidf_intake_down][dd]=0.00005;




           moveconfig[0] [speedg]=0.035;
           moveconfig[0] [strafeg]=0.15;
           moveconfig[0] [turng]=0.03;
           moveconfig[0] [speedmax]=0.4;
           moveconfig[0] [strafemax]=0.7;
           moveconfig[0] [turnmax]=0.2;
           moveconfig[0] [xdis]=-18;
           moveconfig[0] [ydis]=0;
           moveconfig[0] [adis]=0;
           moveconfig[0] [time]=2000;
//strafe  36
           moveconfig[1] [speedg]=0.035;
           moveconfig[1] [strafeg]=0.15;
           moveconfig[1] [turng]=0.03;
           moveconfig[1] [speedmax]=0.4;
           moveconfig[1] [strafemax]=0.7;
           moveconfig[1] [turnmax]=0.2;
           moveconfig[1] [xdis]=0;
           moveconfig[1] [ydis]=37;
           moveconfig[1] [adis]=0;
           moveconfig[1] [time]=3000;
//forward 28
           moveconfig[2] [speedg]=0.035;
           moveconfig[2] [strafeg]=0.15;
           moveconfig[2] [turng]=0.03;
           moveconfig[2] [speedmax]=0.4;
           moveconfig[2] [strafemax]=0.7;
           moveconfig[2] [turnmax]=0.2;
           moveconfig[2] [xdis]=-28;
           moveconfig[2] [ydis]=0;
           moveconfig[2] [adis]=0;
           moveconfig[2] [time]=3000;

           //strafe  12

           moveconfig[3][speedg]=0.035;
           moveconfig[3] [strafeg]=0.15;
           moveconfig[3] [turng]=0.03;
           moveconfig[3] [speedmax]=0.4;
           moveconfig[3] [strafemax]=0.7;
           moveconfig[3] [turnmax]=0.2;
           moveconfig[3] [xdis]=0;
           moveconfig[3] [ydis]=12;
           moveconfig[3] [adis]=0;
           moveconfig[3] [time]=1500;

           //backward 44
           moveconfig[4] [speedg]=0.035;
           moveconfig[4] [strafeg]=0.15;
           moveconfig[4] [turng]=0.03;
           moveconfig[4] [speedmax]=0.4;
           moveconfig[4] [strafemax]=0.7;
           moveconfig[4] [turnmax]=0.2;
           moveconfig[4] [xdis]=44;
           moveconfig[4] [ydis]=0;
           moveconfig[4] [adis]=0;
           moveconfig[4] [time]=3000;
           //forward 44
           moveconfig[5] [speedg]=0.05;//0.035
           moveconfig[5] [strafeg]=0.15;
           moveconfig[5] [turng]=0.03;
           moveconfig[5] [speedmax]=0.5;
           moveconfig[5] [strafemax]=0.7;
           moveconfig[5] [turnmax]=0.2;
           moveconfig[5] [xdis]=-44;
           moveconfig[5] [ydis]=0;
           moveconfig[5] [adis]=0;
           moveconfig[5] [time]=2000;

           //strafe  8

           moveconfig[6] [speedg]=0.035;
           moveconfig[6] [strafeg]=0.15;
           moveconfig[6] [turng]=0.03;
           moveconfig[6] [speedmax]=0.4;
           moveconfig[6] [strafemax]=0.7;
           moveconfig[6] [turnmax]=0.2;
           moveconfig[6] [xdis]=0;
           moveconfig[6] [ydis]=8;
           moveconfig[6] [adis]=0;
           moveconfig[6] [time]=1000;


           //backward 44
           moveconfig[7] [speedg]=0.035;
           moveconfig[7] [strafeg]=0.15;
           moveconfig[7] [turng]=0.03;
           moveconfig[7] [speedmax]=0.4;
           moveconfig[7] [strafemax]=0.7;
           moveconfig[7] [turnmax]=0.2;
           moveconfig[7] [xdis]=44;
           moveconfig[7] [ydis]=0;
           moveconfig[7] [adis]=0;
           moveconfig[7] [time]=3000;
           //strafe  -42
           moveconfig[8] [speedg]=0.035;
           moveconfig[8] [strafeg]=0.15;
           moveconfig[8] [turng]=0.03;
           moveconfig[8] [speedmax]=0.4;
           moveconfig[8] [strafemax]=0.7;
           moveconfig[8] [turnmax]=0.2;
           moveconfig[8] [xdis]=0;
           moveconfig[8] [ydis]=-42;
           moveconfig[8] [adis]=0;
           moveconfig[8] [time]=3000;

           // strafe 33 for move sample
           moveconfig[9] [speedg]=0.035;
           moveconfig[9] [strafeg]=0.15;
           moveconfig[9] [turng]=0.03;
           moveconfig[9] [speedmax]=0.4;
           moveconfig[9] [strafemax]=0.7;
           moveconfig[9] [turnmax]=0.2;
           moveconfig[9] [xdis]=0;
           moveconfig[9] [ydis]=33;
           moveconfig[9] [adis]=0;
           moveconfig[9] [time]=3000;
            // strafe -48 to outtake 3rd
           moveconfig[10] [speedg]=0.035;
           moveconfig[10] [strafeg]=0.15;
           moveconfig[10] [turng]=0.03;
           moveconfig[10] [speedmax]=0.4;
           moveconfig[10] [strafemax]=0.7;
           moveconfig[10] [turnmax]=0.2;
           moveconfig[10] [xdis]=0;
           moveconfig[10] [ydis]=-48;
           moveconfig[10] [adis]=0;
           moveconfig[10] [time]=3000;

           // strafe 45 to intake 4th

           moveconfig[11] [speedg]=0.035;
           moveconfig[11] [strafeg]=0.15;
           moveconfig[11] [turng]=0.03;
           moveconfig[11] [speedmax]=0.4;
           moveconfig[11] [strafemax]=0.7;
           moveconfig[11] [turnmax]=0.2;
           moveconfig[11] [xdis]=0;
           moveconfig[11] [ydis]=45;
           moveconfig[11] [adis]=0;
           moveconfig[11] [time]=3000;

           // strafe -46 to outtake 4th

           moveconfig[12] [speedg]=0.035;
           moveconfig[12] [strafeg]=0.15;
           moveconfig[12] [turng]=0.03;
           moveconfig[12] [speedmax]=0.4;
           moveconfig[12] [strafemax]=0.7;
           moveconfig[12] [turnmax]=0.2;
           moveconfig[12] [xdis]=0;
           moveconfig[12] [ydis]=-46;
           moveconfig[12] [adis]=0;
           moveconfig[12] [time]=3000;

           // strafe 48  to parking
           moveconfig[13] [speedg]=0.05;
           moveconfig[13] [strafeg]=0.30;
           moveconfig[13] [turng]=0.03;
           moveconfig[13] [speedmax]=0.4;
           moveconfig[13] [strafemax]=0.7;
           moveconfig[13] [turnmax]=0.2;
           moveconfig[13] [xdis]=10;
           moveconfig[13] [ydis]=35;
           moveconfig[13] [adis]=0;
           moveconfig[13] [time]=1000;






         //  rotate[in0]=350;    slide[in0]=600;
           rotate[pin1]=520;    slide[pin1]=1450;
           rotate[in1]=450;     slide[in1]=1500;
           rotate[pin2]=650;    slide[pin2]=2900;
           rotate[in2]=580;     slide[in2]=2940;
           rotate[sin]=470;     slide[sin]=600;
           rotate[sina]=290;    slide[sina]=0;
           rotate[sout]=1500;   slide[sout]=600;
           rotate[souta]=3800;   slide[souta]=0;
           rotate[hang1]=3380;  slide[hang1]=0;
           rotate[hang2]=4000;  slide[hang2]=0;
           rotate[hang3]=250;   slide[hang3]=0;
           rotate[out]=3390;    slide[out]=2950;
           rotate[psout]=1100;  slide[psout] =600;
           rotate[psouta]=3300;  slide[psouta] =0;

           //for limit
//           rotate[pin2]=590;    slide[pin2]=2000;
//           rotate[in2]=510;     slide[in2]=2030;
//           handleIn2=0.46;

           imu.resetYaw();
           return;

       }

       if(tstep==1) {//for telop
           //Handle.setPosition(handleIdle);
//           Claw.setPosition(clawIntake);
//           linearslide(armSlide,slide[idle],armslideV1);
//           rotatetargetPIDF(rotate[idle]);
           idle();
       }
        if(tstep==2) {// for auto
            Intake_handle.setPosition(handleOutSpeca);
//            Claw.setPosition(clawClose);
        //   linearslide(0,armslideV1);
            flag[specimenfirst]=true;

        }
    }

    public void armrotatePIDF()

    {
        rotatePos = -Arm_right.getCurrentPosition();
        slidePos = Slide_top.getCurrentPosition();
        controller.setPID(p,i,d);
        double pid = controller.calculate(rotatePos,rotateTarget);
        double ff = Math.cos(Math.toRadians(rotatePos/ticks_in_degree +rotateStartangle)) * (f + k*slidePos) ;// target
        double power = pid + ff;
        Arm_left.setPower(power);
        Arm_right.setPower(power);// to be changed director.

    }

    public boolean sample_intake() {
        if(!flag[intake_ready]) return false;
        int rot,sli;
        stop_drive();
        if(flag[intake_shift]) {// in1
            rot=rotate[in1];
            sli=slide[in1];
        }
        else

        {
            rot=rotate[in2];
            sli=slide[in2];

        }
           // linearslide(armRotate, rotate, armrotateV2);
            rotatetargetPIDF(rot);
             armrotatePIDF();
          //  linearslide( sli, armslideV3);
            delay(300);

//            Claw.setPosition(clawClose);
            delay(300); // 300

         //   linearslide( slide[idle], armslideV3);

            delay(200); //300
            Intake_handle.setPosition(handleOut1);
            rotatetargetPIDF(rotate[idle]+50);

          //  linearslide(armRotate, armrotatePreout0, armrotateV1);

            timer(0,intake);
            flag[intake_ready]=false;
            return true;

    }

    public boolean specimen_intake() {
        if(!flag[intake_ready]) return false;
        stop_drive();
//        Claw.setPosition(clawClose);
        delay(400);
    //   linearslide(armRotate, armrotatePostinSpec, armslideV3);
         rotatetargetPIDF(rotate[sout]);
        armrotatePIDF();
        move(-0.35);
        delay(300);
        stop_drive();
       // rotatetargetPIDF(armrotatePostinSpec);
      //linearslide(armRotate, armrotatepreoutSpec, armrotateV1);
   //     linearslide(slide[sout],armslideV3);
        Intake_handle.setPosition(handleOutSpec);
        timer(0,intake);
        flag[intake_ready]=false;
        flag[lift] = false;
        return true;

    }


    public boolean lift() {
        if(!flag[lift]) {
           // stop_drive();
           pidfsetting(rotate_outtake,pidf_outtake_up);
            flag[lift]=true;
            timer(0,lift);
            Intake_handle.setPosition(handle_outtake);
            Intake_rot.setPosition(handlerot_intake);
            return  false;
        }
        if(-Arm_right.getCurrentPosition()>2000){
            stop_drive();
            linearslide( slide_outtake, slidev2);
            flag[lift]=false;
            return true;
        }

        return false;
    }

    public boolean specimen_lift(int i) {
        if(!flag[lift]) {

//            linearslide(armrotatepreoutSpec,armrotateV3);
           // linearslide(armslideOutSpec,armslideV3);
            Intake_handle.setPosition(handleOutSpec);
            flag[lift] = false;
            return true;
        }

        return false;
    }




    public void claw(boolean close,boolean check)

    {
        if(close)
        {Intake.setPower(-0.9); delay(400);}

        else {Intake.setPower(0.9);delay (400);}
        Intake.setPower(0);


    }


    public void aouttake() {
        stop_drive();
       // linearslide(Arm_left, armrotateOut1, armrotateV2);
        Intake_handle.setPosition(handleOut1);
        pause(1000);
     //   linearslide(Slide_top, armslideOut1, armslideV3);
      //  linearslide(Arm_left, armrotateOut1, armrotateV2);
        pause(2500);
        move(-0.16);
        if (voltageSensor.getVoltage() > 13){
            pause(900);
        }
        else{
            pause(1000);
        }


        stop_drive();
//        Claw.setPosition(clawOpen);
        pause(1000);
        Intake_handle.setPosition(handleIdle);
        pause(600);
        move(0.16);
        if (voltageSensor.getVoltage() > 13){
            pause(900);
        }
        else{
            pause(1000);
        }

        pause(1000);
        stop_drive();
        //linearslide(Slide_top, 0, armrotateV3);
        pause(1000);
      //  linearslide(Arm_left, 0, armrotateV2);
        pause(3000);
        Intake_handle.setPosition(0);
        }






    public boolean outtake() {
        if(!flag[outtake]) {
            stop_drive();
            claw(false, false);
            Intake_handle.setPosition(handlerot_intake);
            flag[drive] = false;
             delay(250);
            flag[outtake] = true;
            move(0.3);
            timer(0,outtake);
           linearslide(slide[slide_idle],slidev2);
            return false;
        }

        if  (Slide_bot.getCurrentPosition()<slide_rotate){
            flag[drive] = true;
            rotatetargetPIDF(rotate[idle]);
            pidfsetting(rotate_idle,pidf_outtake_down);
            Intake_handle.setPosition(handle_idle);
            flag[outtake]= false;
            intake_level=0;
            stop_drive();
            return true;

        }



       return false;

    }

    public void delay(double time)
    {

        int sleepcounter = (int) (time/25);
        for (int i = 0; i < sleepcounter; i++) {
            armrotatePIDF();
            pause(20);
        }
    }




    public boolean specimen_outtake(int i) {

        stop_drive();
        flag[spicemen0]=true;
      //  linearslide(armRotate,armrotatepostoutSpec,armrotateV3);
      //  rotatetargetPIDF(armrotatepostoutSpec);
      //  armrotatePIDF();
        rotatetargetPIDF(rotate[psout]);
        flag[spicemen0]=false;
        delay(100);
        move(-0.3);
        delay(100);
//        Claw.setPosition(clawOpen);
        delay(300);
        stop_drive();
        rotatetargetPIDF(rotate[idle]);


        Intake_handle.setPosition(handleIdle);
       // linearslide(Slide_top,slide[idle],armslideV3);

        flag[outtake]= false;
        step[outtake]=10;
        flag[intake_shift]=false;


        return true;
    }

    public void movecontrol( double ytar) {

        double angle_gap=0, yaw,atarget =0,atarget_ang=0,xtarget_dis=0;//24.5
        double x1,xtarget=0,target=0,xgap,yrange,xrange,y1,a1;
        boolean stop=false;
        double ygap,agap;
        final double SPEED_GAIN = 0.025; // 0.02  //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double STRAFE_GAIN = 0.05; //0.03  //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        final double TURN_GAIN = 0.05;  //0.015  //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        final double MAX_AUTO_SPEED = 0.36;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_TURN = 0.3;

        updatePoseEstimate();
        xtarget=pose.position.x+xtarget_dis;
        target=pose.position.y+ ytar;
        atarget=imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES))+atarget_ang;
         timer(0,2);
        while (Op.opModeIsActive()&&!timer(2500,2)) {// todo &&!timer3(1200)
            armrotatePIDF();
            updatePoseEstimate();
            x1=pose.position.x;
            y1=pose.position.y;
           // a1=Math.toDegrees(pose.heading.toDouble());
            a1=imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
            xgap=xtarget-x1;
            ygap=target-y1;
            agap=atarget-a1;
            if (Math.abs(ygap)<4) break;
            xrange= Range.clip(xgap * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        //    angle_gap = atarget- imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
            yaw = Range.clip(agap * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            yrange= Range.clip(ygap * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            moveRobot(xrange, yrange , yaw);



        }
        stop_drive();
    }
    public void pmove( int step, boolean strafe) {

        double yaw,atar,gap;
        double x1,xtar,ytar,xgap,yrange,xrange,y1,a1;
        double ygap,agap;
        double SPEED_GAIN = moveconfig[step][speedg]; // 0.02  //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        double STRAFE_GAIN = moveconfig[step][strafeg]; //0.03  //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        double TURN_GAIN = moveconfig[step][turng];  //0.015  //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        double MAX_AUTO_SPEED = moveconfig[step][speedmax];;   //  Clip the approach speed to this max value (adjust for your robot)
        double MAX_AUTO_STRAFE = moveconfig[step][strafemax];;   //  Clip the approach speed to this max value (adjust for your robot)
        double MAX_AUTO_TURN =moveconfig[step][turnmax];;

        updatePoseEstimate();
        x1=pose.position.x;
        y1=pose.position.y;
   //     a1=imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
        xtar=moveconfig[step][xdis]+x1;
        ytar=moveconfig[step][ydis]+y1;
        atar=moveconfig[step][adis];
        timer(0,2);
        while (Op.opModeIsActive()&&!timer(moveconfig[step][time],2)) {// todo &&!timer3(1200)
            armrotatePIDF();
            updatePoseEstimate();
            x1=pose.position.x;
            y1=pose.position.y;
            a1=imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
            xgap=xtar-x1;
            ygap=ytar-y1;
            agap=atar-a1;
           if(strafe) gap=ygap; else gap=xgap;
           if (Math.abs(gap)<2) break;
           xrange= Range.clip(xgap * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
           yaw = Range.clip(agap * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
           yrange= Range.clip(ygap * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
           moveRobot(xrange, yrange , yaw);

        }
        stop_drive();
        delay(100);
    }
    public void aspecimen_outtake() {

             double dis=600;
             rotatetargetPIDF(rotate[souta]);
             armrotatePIDF();
            Intake_handle.setPosition(handleOutSpeca);
            delay(1000);//1000
          move(-0.38);//-0.18
        while (Op.opModeIsActive() && dis>soutadis) {
            armrotatePIDF();
//            dis=reardis.getDistance(DistanceUnit.MM);
        }
        stop_drive();
        flag[spicemen0] = true;
        Intake_handle.setPosition(handlepOutSpeca);
        rotatetargetPIDF(rotate[psouta]);
        flag[spicemen0] = false;
        delay(200);
        move(0.2);
      //  delay(50);
//        Claw.setPosition(clawOpen);
        delay(300);
        stop_drive();
       rotatetargetPIDF(rotate[sina]);
       delay(300);// 500
       //Handle.setPosition(handleInSpec);
        // linearslide(armSlide,slide[idle],armslideV3);
      if(flag[asin]) {
          move(0.7);
          delay(200);
      }
        stop_drive();
    }







}


