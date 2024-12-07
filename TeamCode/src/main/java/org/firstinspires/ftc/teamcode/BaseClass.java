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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
    int lslo=0,lshi=2800;

    int armslidePreSpec = 0;



    double rotateStartangle=0 ;

    int rotateTarget=0,crotateTarget=0;





  //  int armrotateHang1 = 2800,armrotateHang2 = 3500 ,armrotateHang3 = 3900, armrotateHang4 = 325;


    PIDController controller;
    final double ticks_in_degree = 8192/360;
    int rotatePos,slidePos,absrotategap;

    double pid ,power, ff;
    double p = 0.00004, i = 0, d = 0.0001 ,f = 0.12,k = 0.000035; //0.000035
// idle, specman slides=400,rotate 380-,


    double handlePos = 0.05, handleH, handleL, handleStep = 0.05, handleCurrent = 0.05;
    double handleIn1 = 0.44, handleIn2 = 0.475;
    double handleOut1 = 0.21, handleIdle = 0.33;//0.33 0.21
    double handleInSpec = 0.14,handlesina=0.16;
    double handleHang = 0.1;
    double handleOutSpec = 0.235, handleOutSpeca=0,handlepOutSpeca=0.1;

    double clawPos = 0.3, clawStep = 0.05, clawCurrent = 0.3, clawH, clawL;//0.24

    public static boolean baseblue = false, baseright = true;

    double[][] afmoveconfig= new double[20][20];
    int speedg=0,strafeg=1,turng=2,speedmax=3,strafemax=4,turnmax=5,xdis=6,ydis=7,adis=8,time=9;
    double[][] pidftable= new double[20][3];
    int pidf_intake_up=0,pidf_intake_down=1, pidf_outtake_down=2,pidf_outtake_up=3, pidf_intake_idle = 4,
            pidf_hang_up = 5, pidf_hang2 = 6, pidf_hang3 = 7, pidf_outtake_spec = 8,
            pidf_outtake_spec_down = 9, pidf_outtake_spec1 = 10 , pidf_outtake_up2 = 11,
            pidf_intake_spec = 12, pidf_intake_spec2 = 13,pidf_intake_aspec=14;
    int pp=0,ii=1,dd=2;


// 3rd robot special

    int roatate_prein0=275, rotate_in0=-40, rotate_idle=50,rotate_outtake=2300, rotate_spec_in = 4100, rotate_spec_out=1500,rotate_spec_first=1450;//intake rotat could not over 70
    int slide_in0=800,slide_in1=1200,slide_in2=1600,slide_idle=200,slide_outtake=2500,slide_rotate=1300, slide_spec_out = 300;   //rotate_outtake 2350
    int intake_level=0;
    double handle_idle=0.05,handle_intake=0.36,handle_specimen_intake=0.07,handle_outtake=0 ,handle_specimen_outtake = 0.1;
    double handlerot_intake=0.55; // 0,0.3.0.75,1;x





    int slidev0=1000,slidev1=1500,slidev2=1500; //2700
 //   double[] handle = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  //  int idle=0,pin1=1,in1=2,pin2=3,in2=4,sin=5,sout=6,hang1=7,hang2=8,hang3=9,out=10,psout = 11,souta=12,psouta=13,sina=14;




    boolean[] flag = new boolean[]{false,false,false,false, false, false, false,false, false,false, false, false, false, false, false, false, false, false,false, false, false, false, false, false,false, false};
    double[] stoptime = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0};
    int[] step = new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0};
    final int start = 0, spec_check = 1, intake = 2, lift = 3, outtake = 4, intake_shift = 5, button_flip = 6;
    final int intake_adjustment = 7, specimenfirst= 8, adjust = 9, drive = 10, intake_ready = 11, hang = 12, specimen = 13,pause=14;
    final int force = 15,color_check=16, hang0 = 17,intake_done=18,hang2=19,asin=20,push=21, spec = 22,pre_spec=23, pre_samp = 24;
    double soutadis=260 , sinadis=150; //250

    int color_det = 0;









    final double RPS = 2787 * 0.9;




    public BaseClass(LinearOpMode linearOpMode, Pose2d pose) {
        super(linearOpMode.hardwareMap, pose);
        Op = linearOpMode;

    }

    public BaseClass(HardwareMap hardwareMap, Pose2d pose1) {
        super(hardwareMap, pose1);

    }







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







    public boolean color () {
        color_det = 0;
        if (Intake_color.green() > 300){
            color_det = 3; //detect yellow
            return true; // take
        }
        if (Intake_color.red() > 150){
            color_det = 1; //detect red
            if (baseblue) return false; // if blue spit
            else return true; // if red take
        }
        if (Intake_color.blue() > 120){
            color_det = 2; //detect blue
            if (baseblue) return true; // if blue take
            else return false; // if red spit
        }
        return false;
    }
    //forhang
    public void hang() {
        if(!flag[hang0]) return;
        linearslideTq(4600,0.98);
        pidfsetting(1600, pidf_hang3); // Hit arm with low rung //1500

        while ((-Arm_right.getCurrentPosition()) < 1200 && Op.opModeIsActive())
        {pause(50);}
       delay(100);
        linearslideTq(4000,0.98);
        // rbg.delay(1000); //1500
        pidfsetting(2000, pidf_hang2); //1600
        //  rbg.delay(1000);
        linearslideTq(-400,0.98);

        while(Op.opModeIsActive()&&Slide_top.getCurrentPosition() > -350) {delay(25);}


        pidfsetting(1839, pidf_hang2);
        delay(500);
        linearslideTq(6400,0.98);
        while(Op.opModeIsActive() && Slide_top.getCurrentPosition() < 6300){delay(25);}
        pidfsetting(2600, pidf_hang2); // 2700
        delay(1000);
        linearslideTq(6000,0.98);
        delay(1000); // 2000
        pidfsetting(1576, pidf_hang2);
        delay(500);
        linearslideTq(-400,0.98);

        while(Op.opModeIsActive() && Slide_top.getCurrentPosition() > -350){
            delay(25);
        }
        delay(1000);
        linearslideTq(-400,0);
        Arm_left.setPower(0);
        Arm_right.setPower(0);
        pause(100000);

    }










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

    public void pre_hang(){
        if (!flag[hang]){
            linearslide(0,slidev2);
            Intake_rot.setPosition(handlerot_intake);
            Intake_handle.setPosition(handle_idle);
            flag[hang] = true;
            step[hang]=0;
            return;
        }
        if ( step[hang]==0&& Slide_top.getCurrentPosition() < 10 ){
            linearslideTq(0,0);

            Gearbox.setPosition(0.95);
            timer(0,hang);
            step[hang]=1;
            return;
        }

        if(step[hang]==1&&timer(600,hang)) {
            linearslideTq(2500,0.98);
            flag[hang]=false;
            flag[hang0]=true;
            step[hang]=-1;
//            timer(0,hang);;
        }
    }

    public void intake_drop() {

        flag[lift] = false;
        Intake.setPower(-0.6);
        delay(200);
        Intake.setPower(0);
           idle();

        }




    public void movestraight(double power) {

            moveRobot(power, 0 , -imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES))*0.015);
    }



    public boolean pre_spec()
    {
       if(!flag[pre_spec]) {
           Intake_handle.setPosition(handle_specimen_intake);
           Intake_rot.setPosition(handlerot_intake);

           linearslide(slide_idle,slidev2);
//           if (Slide_top.getCurrentPosition() > 600) delay(400* (intake_level+1));
//           pidfsetting(rotate_spec_in - 200,pidf_intake_spec);
           flag[pre_spec]=true;
           step[spec_check]=1;
           //timer(0,pre_spec);
           return false;
       }

       if(step[spec_check]==1&&Slide_top.getCurrentPosition()<300)
       {
           pidfsetting(rotate_spec_in - 200,pidf_intake_spec);
           step[spec_check]=2;
           timer(0,pre_spec);
           return false;
       }

       if (timer(600,pre_spec)&&step[spec_check]==2){
           pidfsetting(rotate_spec_in,pidf_intake_spec2);
           flag[pre_spec]=false;
           step[spec_check]=0;
           return true;
       }

       return false;


    }






    public boolean timer(double period, int i) {

        if (period == 0) {
            stoptime[i] = runtime.milliseconds();
            return false;
        }
        return runtime.milliseconds() - stoptime[i] > period;
    }





    public void intake_throw(){
        stop_drive();
        Intake_handle.setPosition(handle_specimen_intake);
        pidfsetting(rotate_spec_in - 200,pidf_intake_spec);
        linearslide(slide_idle,slidev2);
        delay(600);
        pidfsetting(rotate_spec_in,pidf_intake_spec2);
        Intake.setPower(-0.25); // may lower like -0.2

        delay(400);
        flag[lift] = false;
        Intake.setPower(0);


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
    public boolean pre_intake() {
        if (!flag[pre_samp]){
            Intake.setPower(0);
            Intake_handle.setPosition(handle_intake);
            Intake_rot.setPosition(handlerot_intake);
            if (-(Arm_right.getCurrentPosition()) >3800 )  pidfsetting(roatate_prein0, pidf_intake_spec);
            else {
                pidfsetting(roatate_prein0, pidf_intake_up);
                linearslide(slide_in0, slidev1);
                flag[intake_ready]=true;
                flag[pre_samp] = false;
                intake_level=0;
                timer(0,intake);
                return true;
            }
            flag[pre_samp] = true;
            return false;
        }
        if (-(Arm_right.getCurrentPosition()) < 350){
            pidfsetting(roatate_prein0, pidf_intake_up);

            linearslide(slide_in0, slidev1);

            flag[intake_ready]=true;
            flag[pre_samp] = false;
            intake_level=0;
            timer(0,intake);
            return true;
        }
        return false;



    }

    public void pre_specimen()

    {

        Intake_handle.setPosition(handle_idle); // change later
        Intake_rot.setPosition(handlerot_intake);
        pidfsetting(rotate_spec_in,pidf_intake_spec);
        linearslide(slide_idle,slidev2);

        flag[intake_ready]=true;
        intake_level=0;
        timer(0,intake);
      //  Intake.setPower(0.5);

    }



    public void intake_shift(double sticky,boolean drop)

    {
        if(!drop) {
            if (!timer(500, intake_shift) || Math.abs(sticky) < 0.6) return;

            if (sticky < 0) intake_level++;
            else intake_level--;

            if (intake_level > 2) intake_level = 0;
            else if (intake_level < 0) intake_level = 2;
        }
        timer(0,intake_shift);


        if(intake_level==0) {
            linearslide(slide_in0, slidev1);
           // pidfsetting(roatate_prein0, pidf_intake_up);
            return;
        }
        if(intake_level==1){
            linearslide(slide_in1, slidev1);
          //  pidfsetting(roatate_prein1, pidf_intake_up);
            return;
        }

        if(intake_level==2){
            linearslide(slide_in2, slidev1);
          //  pidfsetting(roatate_prein2, pidf_intake_up);

        }

    }



    public void idle()

    {
        Intake_handle.setPosition(handle_idle);
        Intake_rot.setPosition(handlerot_intake);
        linearslide(slide_idle, slidev0);
        pidfsetting(rotate_idle, pidf_intake_idle);

    }



    public void intake() {

        if(!timer(500,intake)) return;
        timer(0,intake);
        Intake.setPower(1.0);
        pidfsetting(rotate_in0,pidf_intake_down);
        color_det = 0;
        while (Op.opModeIsActive() && color_det == 0 && !timer(1000,intake)){
            delay(25);
            flag[color_check] = color();

        }

        if (!flag[color_check]){
            Intake.setPower(-0.7);
            delay(300);
            Intake.setPower(0);
            pre_intake();
        }
        else{
            Intake_handle.setPosition(handle_outtake);
            Intake_rot.setPosition(handlerot_intake);
            pidfsetting(rotate_idle,pidf_intake_up);
            linearslide(slide_idle,slidev2);
            flag[intake_ready]=false;
            flag[button_flip]=false;
            // intake_level=0;// might drop
            delay(300);
            Intake.setPower(0);


        }


    }
    public void intake_no_color() {

        if(!timer(500,intake)) return;

        timer(0,intake);
        Intake.setPower(1.0);
        pidfsetting(rotate_in0,pidf_intake_down);

        delay(500); // decrease

        Intake_handle.setPosition(handle_outtake);
        Intake_rot.setPosition(handlerot_intake);
        pidfsetting(rotate_idle,pidf_intake_up);
        linearslide(slide_idle,slidev2);
        flag[intake_ready]=false;
        flag[button_flip]=false;
            // intake_level=0;// might drop

        delay(150);
        Intake.setPower(0);




    }

    public boolean intake_specimen() {
        if (!flag[spec]){
            Intake.setPower(0.4);
            move(-0.25);
            delay(300);
            stop_drive();
            Intake.setPower(0);
            pidfsetting(rotate_spec_out,pidf_outtake_spec);//pre postion
            flag[spec] = true;
            step[specimen]=1;
            timer(0,spec);

            return false;
        }

        if (timer(1000,spec)&&step[specimen]==1){
            pidfsetting(rotate_spec_out,pidf_outtake_spec1);
            Intake_handle.setPosition(handle_specimen_outtake); // change value for spec outtake
            step[specimen]=2;
            timer(0,spec);
            return false;

        }

        if (timer(500,spec)&&step[specimen]==2){
            linearslide(slide_spec_out,slidev2);
            Intake_handle.setPosition(handle_specimen_outtake); // change value for spec outtake
            flag[spec] = false;
            flag[intake_ready]=false;
            step[specimen]=0;
            return true;

        }

        return false;


    }

    public void aspec_intake() {


        //Intake_handle.setPosition(handle_idle); // change later
//            Intake_rot.setPosition(handlerot_intake);

           //pidfsetting(rotate_spec_in+300,pidf_intake_aspec);
             move(-0.18);
             delay(200);
            Intake.setPower(0.4);
            delay(300);
            pidfsetting(rotate_spec_out,pidf_outtake_spec);//pre postion.
             move(0.3);
            Intake.setPower(0);
            delay(200);
            Intake_handle.setPosition(handle_specimen_outtake);
           // stop_drive();


    }




    public void aspec_outtake() {

        move(0.3);
       pidfsetting(rotate_spec_out,pidf_outtake_spec1);
       delay(200);
       Intake_handle.setPosition(handle_specimen_outtake);
       linearslide(slide_spec_out,slidev2);
        while (bar_dist.getDistance(DistanceUnit.MM)>300&& Op.opModeIsActive())
        {
            armrotatePIDF();
        }
       stop_drive();
        Intake_handle.setPosition(handle_specimen_outtake - 0.04);
        Intake.setPower(0.8);
        delay(50);

        pidfsetting(rotate_spec_out + 600, pidf_outtake_spec_down);
        delay(300); //250
        Intake.setPower(-0.8);
        stop_drive();
        delay(150); //150
        Intake.setPower(0);
        move(-0.55);
        linearslide(slide_idle, slidev2);
        Intake_handle.setPosition(handle_intake);
        pidfsetting(rotate_spec_in +200,pidf_intake_spec);
    }


    public void rbg(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
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
        Slide_top.setTargetPosition(target); // ne
        Slide_bot.setTargetPosition(target);
        Slide_top.setVelocity(speed);
        Slide_bot.setVelocity(speed);

    }

    public void linearslide2( int target, double power) {

        if (target > lshi || target < lslo) return;
        Slide_top.setTargetPosition(target); // ne
        Slide_bot.setTargetPosition(target);
        Slide_top.setPower(power);
        Slide_bot.setPower(power);

    }

    public void linearslideTq( int target, double power) {

        //if (target > lshi || target < lslo) return;
        Slide_top.setTargetPosition(target); // ne
        Slide_bot.setTargetPosition(target);
        Slide_top.setPower(power);
        Slide_bot.setPower(power);

    }

    public void pidfsetting(int target, int index)

    {
        p=pidftable[index][pp];
        i=pidftable[index][ii];
        d=pidftable[index][dd];
        rotateTarget=target;
        armrotatePIDF();
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





    public void init(int tstep)//

    {
       if(tstep==0) { //for both

           imu.resetYaw();
           controller = new PIDController(p, i, d);
           pause(300);
           Intake_rot.setPosition(handlerot_intake);
           Gearbox.setPosition(0);

           flag[drive] = true;
           pidftable[pidf_intake_up][pp]=0.0027;  pidftable[pidf_intake_up][ii]=0;  pidftable[pidf_intake_up][dd]=0.0001;
           pidftable[pidf_intake_idle][pp]=0.003;  pidftable[pidf_intake_idle][ii]=0;  pidftable[pidf_intake_idle][dd]=0.00008;
           pidftable[pidf_intake_down][pp]=0.002;  pidftable[pidf_intake_down][ii]=0;  pidftable[pidf_intake_down][dd]=0.00005;
           pidftable[pidf_outtake_up][pp]=0.00037;  pidftable[pidf_outtake_up][ii]=0;  pidftable[pidf_outtake_up][dd]=0.000023;
           pidftable[pidf_outtake_up2][pp]=0.0025;  pidftable[pidf_outtake_up2][ii]=0;  pidftable[pidf_outtake_up2][dd]=0.0001;
           pidftable[pidf_intake_spec][pp]=0.0004;  pidftable[pidf_intake_spec][ii]=0;  pidftable[pidf_intake_spec][dd]=0.00004; // turn from 0 degrees to 180, may need to decrease P and increase D
           pidftable[pidf_intake_spec2][pp]=0.0013;  pidftable[pidf_intake_spec2][ii]=0;  pidftable[pidf_intake_spec2][dd]=0.00002;
           pidftable[pidf_outtake_down][pp]=0.0005;  pidftable[pidf_outtake_down][ii]=0;  pidftable[pidf_outtake_down][dd]=0.00008;
           pidftable[pidf_hang_up][pp]=0.0016;  pidftable[pidf_hang_up][ii]=0;  pidftable[pidf_hang_up][dd]=0.0001;
           pidftable[pidf_hang2][pp]=0.002;  pidftable[pidf_hang_up][ii]=0;  pidftable[pidf_hang_up][dd]=0.0001; // p used 0.002
           pidftable[pidf_hang3][pp]=0.004; pidftable[pidf_hang3][ii]=0;  pidftable[pidf_hang3][dd]=0.0000;
           pidftable[pidf_intake_aspec][pp]=0.002;  pidftable[pidf_intake_aspec][ii]=0;  pidftable[pidf_intake_aspec][dd]=0.00001; // turn from 0 degrees to 180, may need to decrease P and increase D

           pidftable[pidf_outtake_spec][pp]=0.00025;  pidftable[pidf_outtake_spec][ii]=0;  pidftable[pidf_outtake_spec][dd]=0.00002; // may need to decrease p in future (large turn)
           pidftable[pidf_outtake_spec1][pp]=0.0016;  pidftable[pidf_outtake_spec1][ii]=0.00001;  pidftable[pidf_outtake_spec1][dd]=0.0000; // small turn maintain spec outtake accuracy
           pidftable[pidf_outtake_spec_down][pp]=0.0075;  pidftable[pidf_outtake_spec_down][ii]=0;  pidftable[pidf_outtake_spec_down][dd]=0.00001;


           // strafe for smaples
           afmoveconfig[0] [speedg]=0.035;
           afmoveconfig[0] [strafeg]=0.15;
           afmoveconfig[0] [turng]=0.018;
           afmoveconfig[0] [speedmax]=0.5;
           afmoveconfig[0] [strafemax]=0.7;
           afmoveconfig[0] [turnmax]=0.2;
           afmoveconfig[0] [xdis]=12;
           afmoveconfig[0] [ydis]=-25;
           afmoveconfig[0] [adis]=0;
           afmoveconfig[0] [time]=2000;

           // forward to move sample
           afmoveconfig[1] [speedg]=0.04;
           afmoveconfig[1] [strafeg]=0.15;
           afmoveconfig[1] [turng]=0.03;
           afmoveconfig[1] [speedmax]=0.8;
           afmoveconfig[1] [strafemax]=0.7;
           afmoveconfig[1] [turnmax]=0.2;
           afmoveconfig[1] [xdis]=45;
           afmoveconfig[1] [ydis]=-29;
           afmoveconfig[1] [adis]=0;
           afmoveconfig[1] [time]=2000;
           // strafe for first sample 10
           afmoveconfig[2] [speedg]=0.02;
           afmoveconfig[2] [strafeg]=0.2;//0.15
           afmoveconfig[2] [turng]=0.03;
           afmoveconfig[2] [speedmax]=0.6;
           afmoveconfig[2] [strafemax]=0.6;
           afmoveconfig[2] [turnmax]=0.2;
           afmoveconfig[2] [xdis]=44;
           afmoveconfig[2] [ydis]=-39;
           afmoveconfig[2] [adis]=0;
           afmoveconfig[2] [time]=3000;
           // push first sample TO -6
           afmoveconfig[3] [speedg]=0.05;
           afmoveconfig[3] [strafeg]=0.15;
           afmoveconfig[3] [turng]=0.02;
           afmoveconfig[3] [speedmax]=0.8;
           afmoveconfig[3] [strafemax]=0.7;
           afmoveconfig[3] [turnmax]=0.2;
           afmoveconfig[3] [xdis]=15;
           afmoveconfig[3] [ydis]=-39;
           afmoveconfig[3] [adis]=0;
           afmoveconfig[3] [time]=2000;
           // back  for second sample 42
           afmoveconfig[4] [speedg]=0.04;
           afmoveconfig[4] [strafeg]=0.15;
           afmoveconfig[4] [turng]=0.03;
           afmoveconfig[4] [speedmax]=0.8;
           afmoveconfig[4] [strafemax]=0.7;
           afmoveconfig[4] [turnmax]=0.2;
           afmoveconfig[4] [xdis]=44;
           afmoveconfig[4] [ydis]=-39;
           afmoveconfig[4] [adis]=0;
           afmoveconfig[4] [time]=2000;
           //strafe for second samples 8
           afmoveconfig[5] [speedg]=0.04;
           afmoveconfig[5] [strafeg]=0.2;//0.15
           afmoveconfig[5] [turng]=0.03;
           afmoveconfig[5] [speedmax]=0.8;
           afmoveconfig[5] [strafemax]=0.7;
           afmoveconfig[5] [turnmax]=0.2;
           afmoveconfig[5] [xdis]=44;
           afmoveconfig[5] [ydis]=-48;
           afmoveconfig[5] [adis]=0;
           afmoveconfig[5] [time]=2000;
           // push second sample
           afmoveconfig[6] [speedg]=0.04;
           afmoveconfig[6] [strafeg]=0.15;
           afmoveconfig[6] [turng]=0.03;
           afmoveconfig[6] [speedmax]=0.8;
           afmoveconfig[6] [strafemax]=0.7;
           afmoveconfig[6] [turnmax]=0.2;
           afmoveconfig[6] [xdis]=15;
           afmoveconfig[6] [ydis]=-48;
           afmoveconfig[6] [adis]=0;
           afmoveconfig[6] [time]=2000;

           //strafe for outtake
           afmoveconfig[7] [speedg]=0.02;
           afmoveconfig[7] [strafeg]=0.15;
           afmoveconfig[7] [turng]=0.02;
           afmoveconfig[7] [speedmax]=0.5;
           afmoveconfig[7] [strafemax]=0.7;
           afmoveconfig[7] [turnmax]=0.25;
           afmoveconfig[7] [xdis]=-8;
           afmoveconfig[7] [ydis]=0;
           afmoveconfig[7] [adis]=0;
           afmoveconfig[7] [time]=2000;

           //strafe for intake
           afmoveconfig[8] [speedg]=0.02;
           afmoveconfig[8] [strafeg]=0.15;
           afmoveconfig[8] [turng]=0.02;
           afmoveconfig[8] [speedmax]=0.5;
           afmoveconfig[8] [strafemax]=0.7;
           afmoveconfig[8] [turnmax]=0.25;
           afmoveconfig[8] [xdis]=-12;
           afmoveconfig[8] [ydis]=36;
           afmoveconfig[8] [adis]=0;
           afmoveconfig[8] [time]=2000;
           //strafe for outtake
           afmoveconfig[9] [speedg]=0.02;
           afmoveconfig[9] [strafeg]=0.15;
           afmoveconfig[9] [turng]=0.02;
           afmoveconfig[9] [speedmax]=0.5;
           afmoveconfig[9] [strafemax]=0.7;
           afmoveconfig[9] [turnmax]=0.25;
           afmoveconfig[9] [xdis]=-8;
           afmoveconfig[9] [ydis]=8;
           afmoveconfig[9] [adis]=0;
           afmoveconfig[9] [time]=2000;

           //strafe for intake
           afmoveconfig[10] [speedg]=0.02;
           afmoveconfig[10] [strafeg]=0.15;
           afmoveconfig[10] [turng]=0.02;
           afmoveconfig[10] [speedmax]=0.5;
           afmoveconfig[10] [strafemax]=0.7;
           afmoveconfig[10] [turnmax]=0.2;
           afmoveconfig[10] [xdis]=-12;
           afmoveconfig[10] [ydis]=36;
           afmoveconfig[10] [adis]=0;
           afmoveconfig[10] [time]=2000;
           //strafe for outtake
           afmoveconfig[11] [speedg]=0.02;
           afmoveconfig[11] [strafeg]=0.15;
           afmoveconfig[11] [turng]=0.02;
           afmoveconfig[11] [speedmax]=0.5;
           afmoveconfig[11] [strafemax]=0.7;
           afmoveconfig[11] [turnmax]=0.25;
           afmoveconfig[11] [xdis]=-8;
           afmoveconfig[11] [ydis]=-2;
           afmoveconfig[11] [adis]=0;
           afmoveconfig[11] [time]=2000;
           //strafe for parking
           afmoveconfig[12] [speedg]=0.02;
           afmoveconfig[12] [strafeg]=0.15;
           afmoveconfig[12] [turng]=0.02;
           afmoveconfig[12] [speedmax]=0.5;
           afmoveconfig[12] [strafemax]=0.7;
           afmoveconfig[12] [turnmax]=0.2;
           afmoveconfig[12] [xdis]=-8;
           afmoveconfig[12] [ydis]=38;
           afmoveconfig[12] [adis]=0;
           afmoveconfig[12] [time]=2000;


           return;

       }

       if(tstep==1) {//for telop
           //Handle.setPosition(handleIdle);
//           Claw.setPosition(clawIntake);
//           linearslide(armSlide,slide[idle],armslideV1);
//           rotatetargetPIDF(rotate[idle]);
           idle();
       }
        if(tstep==2) {// auto rest the roate 0 position.
            Arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Arm_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           Arm_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           return;
        }

        if(tstep==3) {// for auto size limitation


            Slide_bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slide_bot.setTargetPosition(0);
            Slide_bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide_bot.setVelocity(0);

            Slide_top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slide_top.setTargetPosition(0);
            Slide_top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide_top.setVelocity(0);
            pause(600);
            pidfsetting(-Arm_right.getCurrentPosition(),pidf_intake_idle);
            delay(500);
            return;

        }


        if(tstep==4) {//first outake

            move(0.25);
            delay(100);
            Intake_handle.setPosition(0.15);
            Intake_rot.setPosition(handlerot_intake);
            pidfsetting(rotate_spec_first+200,pidf_intake_spec);
            delay(400);
            pidfsetting(rotate_spec_first,pidf_intake_down);
            delay(300);
            linearslide(550,slidev1);// first outtake

         while(Op.opModeIsActive()&& bar_dist.getDistance(DistanceUnit.MM)>270)
            {
                movestraight(0.3);
                armrotatePIDF();

            }
         stop_drive();
        ;
         delay(100000000);
         Intake.setPower(0.8);
         Intake_handle.setPosition(0.28);
         pidfsetting(rotate_spec_first-500,pidf_outtake_spec_down);
            delay(150); //250
            Intake.setPower(-0.9);
            delay(150); //150
            Intake.setPower(0);
            move(-0.4);
          delay(150);

        }



        if(tstep==5) {


            pidfsetting(rotate_spec_out,pidf_outtake_up2);
            delay(500);
            linearslide(0,slidev0);
            delay(1000);
            pidfsetting(rotate_spec_in,pidf_outtake_up2);
            delay(1000);





        }
    }

    public void afmove( int step, boolean str) {

        double yaw,atar;//,gap;
        double x1,xtar,ytar,yrange,xrange,y1,a1;
        double ygap=0,agap=0,xgap=0;
        double SPEED_GAIN = afmoveconfig[step][speedg]; // 0.02  //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        double STRAFE_GAIN = afmoveconfig[step][strafeg]; //0.03  //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        double TURN_GAIN = afmoveconfig[step][turng];  //0.015  //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        double MAX_AUTO_SPEED = afmoveconfig[step][speedmax];;   //  Clip the approach speed to this max value (adjust for your robot)
        double MAX_AUTO_STRAFE = afmoveconfig[step][strafemax];;   //  Clip the approach speed to this max value (adjust for your robot)
        double MAX_AUTO_TURN =afmoveconfig[step][turnmax];;
        xtar=afmoveconfig[step][xdis];
        ytar=afmoveconfig[step][ydis];
        atar=afmoveconfig[step][adis];
//        if(flag[asup]) {
//            rotatetargetPIDF(rotate[soutb]);
//            flag[asup]=false;
//        }

        timer(0,6);
        while (Op.opModeIsActive()&&!timer(afmoveconfig[step][time],6)) {// todo &&!timer3(1200)
            armrotatePIDF();
            updatePoseEstimate();
            x1=pose.position.x;
            y1=pose.position.y;
            a1=imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
            xgap=xtar-x1;
            ygap=ytar-y1;
            agap=atar-a1;
            if(str) {if (Math.abs(ygap)<2) break;}
            else {if(Math.abs(xgap)<2) break;}


            xrange= Range.clip(xgap * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            yaw = Range.clip(agap * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            yrange= Range.clip(ygap * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            moveRobot(xrange, yrange , yaw);

        }
        stop_drive();
        //  delay(30);//100
    }

    public void afirst_spec_outake(){

        {//first outake
            double dis=500;
            move(0.25);
            delay(100);
            Intake_handle.setPosition(0.13);
            Intake_rot.setPosition(handlerot_intake);
            pidfsetting(rotate_spec_first+200,pidf_intake_spec);
            delay(400);
            pidfsetting(rotate_spec_first,pidf_intake_down);
            delay(200);
            linearslide(580,slidev2);// first outtake

            while(Op.opModeIsActive()&& dis>205)
            {
                movestraight(0.30);
                armrotatePIDF();
                dis=bar_dist.getDistance(DistanceUnit.MM);
                if(!flag[specimen]&& dis <310)
                {Intake.setPower(0.8);flag[specimen]=true;}
            }

            Intake.setPower(0.8);
            stop_drive();
         //  delay(100000000);
           //// Intake.setPower(0.8);
            Intake_handle.setPosition(0.20);
            pidfsetting(rotate_spec_first-400,pidf_outtake_spec_down);
            delay(400); //250
            Intake.setPower(-0.9);
            delay(100); //150
            Intake.setPower(0);
            move(-0.4);
            delay(100);
            linearslide(slide_idle, slidev2);
            delay(250);
            Intake_handle.setPosition(handle_idle); // change later
            Intake_rot.setPosition(handlerot_intake);
            pidfsetting(rotate_spec_in+200,pidf_intake_spec);
            stop_drive();
            delay(100);
            flag[specimen]=false;

        }

    }


    public void armrotatePIDF()

    {
        rotatePos = -Arm_right.getCurrentPosition();
        slidePos = Slide_top.getCurrentPosition();
        controller.setPID(p,i,d);
        pid = controller.calculate(rotatePos,rotateTarget);
        ff = Math.cos(Math.toRadians(rotatePos/ticks_in_degree +rotateStartangle)) * (f + k*slidePos) ;// target
        power = pid + ff;

        if (power > 1.0) power = 0.98;
        if (power < -1.0) power = -0.98;
        Arm_left.setPower(-power);
        Arm_right.setPower(power);// to be changed director.

    }





    public boolean lift() {
        if(!flag[lift]) {
           // stop_drive();
           pidfsetting(rotate_outtake,pidf_outtake_up);
            flag[lift]=true;
            timer(0,lift);
            Intake_handle.setPosition(handle_outtake);
            Intake_rot.setPosition(handlerot_intake);
            return false;
        }
        if(-(Arm_right.getCurrentPosition())>2000){
            stop_drive();
            pidfsetting(rotate_outtake,pidf_outtake_up2);
            linearslide(slide_outtake,slidev2);
            flag[lift]=false;
            return true;
        }

        return false;
    }






    public void claw(boolean close,boolean check)

    {
        if(close)
        {Intake.setPower(0.9); delay(1000);}

        else {Intake.setPower(-0.2);delay (300);} // -0.2
        Intake.setPower(0);


    }





    public boolean outtake() {
        if(!flag[outtake]) {
            stop_drive();
            claw(false, false);
            Intake_handle.setPosition(handlerot_intake);
            flag[drive] = false;
             delay(250);
            flag[outtake] = true;
            move(0.25);
            timer(0,outtake);
            linearslide(slide_idle,slidev2);
//           linearslide(slide_idle,slidev2);
            return false;
        }

        if  (Slide_bot.getCurrentPosition()<slide_rotate){
            flag[drive] = true;
            linearslide(slide_idle,slidev0);
            pidfsetting(rotate_idle,pidf_outtake_down);
            Intake_handle.setPosition(handle_idle);
            flag[outtake]= false;
            intake_level=0;
            stop_drive();
            return true;

        }



       return false;

    }

    public boolean outtake_spec() {
        if (!flag[outtake]){
            stop_drive();
            flag[drive] = false;
            Intake.setPower(0.8);


            Intake_handle.setPosition(handle_specimen_outtake - 0.04);
//            linearslide(slide_spec_out + 400, slidev2);
            pidfsetting(rotate_spec_out + 600,pidf_outtake_spec_down ); //650
            delay(300); //200

            Intake.setPower(-0.9);
            stop_drive();
            delay(200); //150
            Intake.setPower(0);
            move(-0.8);
            flag[outtake] = true;
            linearslide(slide_idle,slidev2);
            delay(250);

            flag[drive] = true;
            pidfsetting(rotate_idle,pidf_outtake_down);
            Intake_handle.setPosition(handle_idle);
            flag[outtake] = false;
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











}


