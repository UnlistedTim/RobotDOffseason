package org.firstinspires.ftc.teamcode;



import com.arcrobotics.ftclib.controller.PIDController;

import com.acmerobotics.roadrunner.Pose2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.Range;
;
import com.qualcomm.robotcore.util.ElapsedTime;

import   java.lang.Math;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class BaseClass extends MecanumDrive {

    // Ticks from old rpm to new rpm for slide factor 0.4769124742


    public ElapsedTime runtime = new ElapsedTime();
    public LinearOpMode Op;

   // double intakerotpose=0,gearboxpose=0;

    // p 0.00012 down: spec outtake to intake idle (100) and sample outtake to intake idle (100)

    // p 0.0002 up: intake idle (100) to spec outtake and intake idle to sample outtake
    double speed_index=1;
    double arm_angle=0;
    double claw_close=0.93,claw_open=0.72;
    double arm_angle_target;
    double arm_angle_idle=0,arm_angle_preintake=5,arm_arngle_intake=0,arm_angle_sampleouttake=107,arm_angle_specintake=197,arm_angle_specouttake=40;
    double lefthandle_idle=0.58,lefthandle_intake=0.28,lefthandle_left45=0.24,lefthandle_left90=0.14,lefthandle_right45=0.34,lefthandle_right90=0.42;
         double lefthandle_sampleouttake=0.66,lefthandle_specintake=0.48,lefthandle_specouttake=0.96,lefthandle_start=0;

    double righthandle_idle=0.42,righthandle_intake=0.72,righthandle_left45=0.68,righthandle_left90=0.58,righthandle_right45=0.78,righthandle_right90=0.86;
    double righthandle_sampleouttake=0.34,righthandle_specintake=0.52,righthandle_specouttake=0.64,righthandle_start=1;

    int slide_idle=200,slide_preintake=400,slide_sampleouttake=1750,slide_specintake=0,slide_specouttake=720,slide_intakemax=1250;

    int  slide_rotate=450,lslo=-5,lshi=1850;


    PIDController controller;

    int rotatePos,slidePos;
    double pid ,power, ff;
    double p = 0.00004, i = 0, d = 0.0001 ,f = 0.12,k = 0.000035; //0.000035
    double handlePos = 0.05, handleStep = 0.05;
    double  clawStep = 0.05;
    public static boolean baseblue = false, baseright = true,baserest;

    double[][] afmoveconfig= new double[20][20];
    double[][] asconfig= new double[10][10];
    int speedg=0,strafeg=1,turng=2,speedmax=3,strafemax=4,turnmax=5,xdis=6,ydis=7,adis=8,time=9;
    double[][] pidftable= new double[30][3];
    int pidf_intake_up=20,pidf_intake=1,pidf_sampleouttake=2 ,pidf_outtake_down=30,pidf_outtake_up=3, pidf_intake_idle = 4,
            pidf_hang_up = 5, pidf_hang2 = 6, pidf_hang3 = 7, pidf_outtake_spec = 8,
            pidf_outtake_spec_down = 9, pidf_outtake_spec1 = 10 , pidf_outtake_up2 = 11,
            pidf_intake_spec = 12, pidf_intake_spec2 = 13,pidf_intake_aspec=14,pidf_aspec_outtake=15,pidf_outtake_aspec_down=16,pidf_aintake_down=17, pidf_hang4 = 18,pidf_specintake=20;
    int  pidf_specouttake;
    int pp=0,ii=1,dd=2;
    double xo,yo,ao;

    int roatate_prein0=415, rotate_in0=-40, rotate_idle=30,rotate_outtake=2400, rotate_spec_in = 4100, rotate_spec_out=1500,rotate_spec_first=1450;//intake rotat could not over 70
    int slide_in0=800,slide_in1=1200,slide_in2=1600,slide_outtake=2470, slide_spec_out = 350;   //rotate_outtake 2350
    int intake_level=0;
    int slidev0=1000,slidev1=1500,slidev2=2700; //2700

    boolean[] flag = new boolean[]{false,false, false,false,false,false,false, false, false, false,false,false,false,false, false,false, false,false,false, false, false,false,false, false,false, false, false, false,false,false,false, false, false,false, false, false, false, false, false,false, false};
    double[] stoptime = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0};
    int[] step = new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0};
    final int start = 0, spec_check = 1, intake = 2, lift = 3, outtake = 4, intake_shift = 5, button_flip = 6;
    final int intake_adjustment = 7, intake_slide= 8, last = 9, drive = 10, intake_ready = 11, hang = 12, specimen = 13,smooth_adj=14;
    final int force = 15,color_check=16, hang0 = 17,idle_ready=18,spec_ready=19,preidle=20,idleready=21, spec = 22,pre_spec=23, pre_samp = 24,presampleintake=25,sampleintakeready=26;
    final int preintakeidle=27,intakeidleready=28,presampleouttake=29,sampleouttakeready=30,presamplelift=31,sampleliftready=32,prespecintake=33,specintakeready=34,placement=35,prespecouttake=36,specouttakeready=37;
    final  double arm_angle_offset=160;
    int color_det = 0;





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
        if(!flag[hang0] || Slide_top.getCurrentPosition() < 4000 ) return;
        k=k/6;
        move(0.3);
        linearslideTq(4600,0.98);
        pidfsetting(1600, pidf_hang3); // Hit arm with low rung //1500

        while ((-Arm_right.getCurrentPosition()) < 1200 && Op.opModeIsActive())
        {delay(25);}
       delay(100);

        linearslideTq(4000,0.98);
    // rbg.delay(1000); //1500
        pidfsetting(2000, pidf_hang2); //1600
        //  rbg.delay(1000);
        linearslideTq(-650,0.98);
        stop_drive();

        while(Op.opModeIsActive()&&Slide_top.getCurrentPosition() > -550) {delay(25);}


        pidfsetting(1839, pidf_hang2);
        delay(100);
        linearslideTq(6800,0.98);
        while(Op.opModeIsActive() && Slide_top.getCurrentPosition() < 5000){delay(25);}
        pidfsetting(2800, pidf_hang4); // 2700
        delay(1000);
        linearslideTq(6000,0.98);
        delay(500); // 2000
        pidfsetting(1576, pidf_hang2);
        linearslideTq(-600,0.98);

        while(Op.opModeIsActive() && Slide_top.getCurrentPosition() > -450){
            delay(25);
        }
        delay(1000);
        linearslideTq(-600,0);
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
//            Intake_rot.setPosition(handlerot_intake);
//            Intake_handle.setPosition(handle_idle);
            flag[hang] = true;
            step[hang]=0;
            return;
        }
        if ( step[hang]==0&& Slide_top.getCurrentPosition() < 10 ){
            linearslideTq(0,0);

            Gearbox.setPosition(0.95);
            pidfsetting(600,pidf_hang_up);
            timer(0,hang);
            step[hang]=1;
            return;
        }

        if(step[hang]==1&&timer(600,hang)) {
            linearslideTq(4600,0.98);

            flag[hang]=false;
            flag[hang0]=true;
            step[hang]=-1;
//            timer(0,hang);;
        }
    }

    public boolean intake_drop() {

        if (!timer(4000,intake)){
//            Intake.setPower(-0.6);
            delay(200);
            pre_intake();
            speed_index=0.4;
            return true;

        }


        if(!flag[idle_ready]) {
            speed_index=1;
            stop_drive();
            flag[lift] = false;
//            Intake.setPower(-0.6);
//            delay(200);
//            Intake.setPower(0);
//            Intake_handle.setPosition(handle_idle);
//            Intake_rot.setPosition(handlerot_intake);
            linearslide(slide_idle, slidev1);
            flag[idle_ready]=true;
            return false;

        }

        if(Slide_top.getCurrentPosition()<400) {
            pidfsetting(rotate_idle, pidf_intake_spec2);
            flag[idle_ready] = false;
            flag[intake_ready] = false;
            flag[pre_samp] = false;
            return true;

        }

        return false;

    }




    public void movestraight(double power) {

            moveRobot(power, 0 , -imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES))*0.015);
    }



    public boolean pre_spec()
    {
       if(!flag[pre_spec]) {
//           Intake_handle.setPosition(handle_specimen_intake);
//           Intake_rot.setPosition(handlerot_intake);
           linearslide(slide_idle,slidev2);
//           if (Slide_top.getCurrentPosition() > 600) delay(400* (intake_level+1));
//           pidfsetting(rotate_spec_in - 200,pidf_intake_spec);
           flag[pre_spec]=true;
           step[spec_check]=1;
           //timer(0,pre_spec);
           return false;
       }

       if(Slide_top.getCurrentPosition()<300)
       {
           pidfsetting(rotate_spec_in+140 ,pidf_intake_spec);
           flag[spec] = false;
           return true;
       }

//       if (timer(600,pre_spec)&&step[spec_check]==2){
//           pidfsetting(rotate_spec_in,pidf_intake_spec2);
//
//
//           return true;
//       }

       return false;


    }


    public void pre_spec_intake()
    {
//            Intake_handle.setPosition(handle_specimen_intake);
//            Intake_rot.setPosition(handlerot_intake);
            linearslide(slide_idle,slidev2);
            flag[spec_ready]=false;
    }
    public void spec_intak_ready()
    {
        if(Slide_top.getCurrentPosition()<300)
        {
            pidfsetting(rotate_spec_in+140 ,pidf_intake_spec);
            flag[spec_ready] = true;

        }

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
//        Intake_handle.setPosition(handle_specimen_intake);
        pidfsetting(rotate_spec_in+140 ,pidf_intake_spec);
        linearslide(slide_idle,slidev2);
        delay(600);
       // pidfsetting(rotate_spec_in,pidf_intake_spec2);
//        Intake.setPower(-0.25); // may lower like -0.2

        flag[pre_spec]=true;
        delay(300);
        flag[lift] = false;
//        Intake.setPower(0);
        flag[spec] = false;

    }


      public void pre_intake_adjust(double adjustx) {
        if(!timer(300,intake_adjustment)) return;
        if(Math.abs(adjustx)<0.7) return;
//        double tar=Intake_rot.getPosition();
//        timer(0,intake_adjustment);
//        if(adjustx<-0.7)  {Intake_rot.setPosition(tar-0.25);return;}
//        if(adjustx>0.7)  {Intake_rot.setPosition(tar+0.25);;}


    }

    public void  intake_smooth_adjustst(double sticky) {

        if(sticky<-0.5) {linearslide(slide_in2,slidev0) ;flag[smooth_adj]=true;return ;}
        if(sticky>0.5)  {linearslide(slide_in0,slidev0); flag[smooth_adj]=true;return;}
        if(flag[smooth_adj]) {
            linearslide(Slide_top.getCurrentPosition(),slidev0-200);
            flag[smooth_adj]=false;}
    }


    public void pre_intake() {
//            Intake_handle.setPosition(handle_intake);
//            Intake_rot.setPosition(handlerot_intake);
            pidfsetting(roatate_prein0, pidf_intake_up);
            linearslide(slide_in0, slidev1);
            flag[intake_ready]=true;
            flag[pre_samp] = false;
            intake_level=0;
            timer(0,intake);
//            Intake.setPower(0);
        }






    public void pre_specimen()

    {

//        Intake_handle.setPosition(handle_idle); // change later
//        Intake_rot.setPosition(handlerot_intake);
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



//    public boolean stepidle()
//
//    {
//       if(!flag[idle_ready]) {
////           Intake_handle.setPosition(handle_idle);
////           Intake_rot.setPosition(handlerot_intake);
//           linearslide(slide_idle, slidev1);
//           flag[idle_ready]=true;
//          return false;
//
//       }
//
//
//        if(Slide_top.getCurrentPosition()<400) {
//            pidfsetting(rotate_idle, pidf_intake_spec2);
//            flag[idle_ready] = false;
//            flag[intake_ready]=false;
//            flag[pre_samp] = false;
//            flag[pre_spec]=false;
//            return true;
//        }
//
//        return false;
//
//    }

    public boolean drop()

    {
     Claw.setPosition(claw_open);
     delay(300);
     if(timer(5000,intake))  {pre_idle(); return true;}

     pre_sampleintake();
     return false;
    }


    public void pre_idle()

    {
        linearslide(slide_idle, slidev2);
        flag[preidle]=true;
        flag[idle_ready]=false;
    }

    public void idle_ready()

    {

        if(flag[preidle]) {

            if(speed_index<1 &&Slide_top.getCurrentPosition()<800) speed_index=1;
            if (Slide_top.getCurrentPosition() < slide_rotate) {  //slide roataiton target
                pidfsetting(arm_angle_idle, 0);
                flag[preidle] = false;

            }
            return;
        }

       if (Math.abs(arm_angle-arm_angle_idle)<15)
       {
           Left_handle.setPosition(lefthandle_idle);
           Right_handle.setPosition(righthandle_idle);
           flag[idleready]=true;
       }

    }

    public void pre_sampleintake()

    {
        linearslide(slide_preintake,slidev2);
        flag[button_flip]=false;
        flag[presampleintake]=true;
        flag[sampleintakeready]=false;

    }


    public void sampleintake_ready(boolean pad2rbumperpress)

    {
        if(!flag[button_flip] && !pad2rbumperpress) flag[button_flip]=true;
        if(flag[presampleintake]&&Slide_top.getCurrentPosition() <slide_rotate){  //slide roataiton target
            pidfsetting(arm_angle_preintake, pidf_intake);
            flag[presampleintake]=false;
        }

        if (Math.abs(arm_angle-arm_angle_preintake)<15)
        {
            Claw.setPosition(claw_open);
            Left_handle.setPosition(lefthandle_intake);
            Right_handle.setPosition(righthandle_intake);
           if(flag[button_flip]) flag[sampleintakeready]=true;
        }


    }

    public void pre_specintake(boolean driver) {

        flag[placement]=false;
        linearslide(slide_specintake, slidev2);
        flag[prespecintake] = true;
        flag[specintakeready] = false;
        if(driver) flag[placement]=true;

    }


    public void specintake_ready()

    {

        if(flag[presampleintake]&&Slide_top.getCurrentPosition() <slide_rotate){  //slide roataiton target
            pidfsetting(arm_angle_preintake-10, 0);
            flag[presampleintake]=false;
        }

        if (Math.abs(arm_angle-arm_angle_preintake)<15)
        {
          //  Claw.setPosition(claw_open);
            Left_handle.setPosition(lefthandle_specintake);
            Right_handle.setPosition(righthandle_specintake);

            pidfsetting(arm_angle_preintake, pidf_specintake);
            flag[specintakeready]=true;
        }


    }



    public void specplacment(){
        stop_drive();
        if(Claw.getPosition()>0.8)
        {
            Claw.setPosition(claw_open);
            delay(100);
        }
        flag[placement]=false;
       // delay(300);

    }
    public void specintake() {

            stop_drive();
            move(-0.25);
            delay(300);
            stop_drive();
            Claw.setPosition(claw_close);
            delay(250);


        }


    public void pre_specouttake() {

        pidfsetting(arm_angle_specouttake+10,0);
        delay(150);
        move(0.25);
        delay(100);
        stop_drive();
        flag[prespecouttake] = true;
        flag[specouttakeready] = false;


    }

    public void specouttake_ready()

    {

        if(flag[prespecouttake]&& (Math.abs(arm_angle_update()-arm_angle_specouttake)<20)) {
           linearslide(slide_specouttake,slidev2);
            Left_handle.setPosition(lefthandle_specouttake);
            Right_handle.setPosition(righthandle_specouttake);
            flag[prespecouttake]=false;
            return;
        }

        if (Math.abs(Slide_top.getCurrentPosition()-slide_specouttake)<15)
        {
            pidfsetting(arm_angle_specouttake,pidf_specouttake);
            flag[specintakeready]=true;
        }


    }
    public void specouttake() {

        move(0.4);
        while(bar_dist.getDistance(DistanceUnit.MM)>200)
        {
            armrotatePIDF();
        }
        stop_drive();
        Claw.setPosition(claw_open);
        delay(150);
        move(-0.3);
        delay(200);
        stop_drive();

    }


    public void pre_intakeidle()

    {

//        Intake_handle.setPosition(handle_idle);
        //Intake_rot.setPosition(handlerot_idle);
        //set claw;
        linearslide(slide_idle, slidev1);
        flag[preidle]=true;
    }


    public void  intakeidle_ready()

    {
        if(Slide_top.getCurrentPosition() <slide_rotate&&flag[preintakeidle]){  //slide roataiton target
           // pidfsetting(rotate_idle, pidf_intake);
            flag[preintakeidle]=false;
            flag[intakeidleready]=true;
            flag[lift]=false;
        }

    }

    public void pre_samplelift(boolean driver)

    {
        flag[lift]=false;
        linearslide(slide_idle,slidev2);
        flag[presamplelift]=true;
        flag[sampleliftready]=false;
        if(driver) flag[lift]=true;


    }


    public void samplelift_ready()

    {

        if(flag[presampleintake]&&Slide_top.getCurrentPosition() <slide_rotate){  //slide roataiton target
            pidfsetting(arm_angle_sampleouttake-10 , 0);
            flag[presampleintake]=false;
        }

        if (Math.abs(arm_angle-arm_angle_update()-arm_angle_preintake)<18)
        {

            Left_handle.setPosition(lefthandle_sampleouttake);
            Right_handle.setPosition(righthandle_sampleouttake);
            flag[sampleouttakeready]=true;
        }


    }

    public void pre_sampleouttake()

    {

        linearslide(slide_sampleouttake,slidev2);
        pidfsetting(arm_angle_sampleouttake , pidf_sampleouttake);
        flag[presampleouttake]=true;
        flag[sampleliftready]=false;
        speed_index=0.3;
        flag[lift]=false;

    }


    public void sampleouttake_ready()

    {

        if(Slide_top.getCurrentPosition() >(slide_sampleouttake-15)){  //slide roataiton target

            flag[presampleintake]=false;
            flag[sampleouttakeready]=true;
        }


    }
    public  void sampleouttake() {

            stop_drive();
            Claw.setPosition(claw_open);
            delay(200);
            Left_handle.setPosition(lefthandle_idle);
            Right_handle.setPosition(righthandle_idle);
            delay(150);
            move(0.3);
            delay(100);
           // linearslide(slide_idle,slidev2);
            //flag[drive] = false;
            stop_drive();

    }








    public void directdle()

    {

//            Intake_handle.setPosition(handle_idle);
//            Intake_rot.setPosition(handlerot_intake);
            pidfsetting(rotate_idle, pidf_intake_spec2);
            linearslide(slide_idle, slidev1);
            flag[idle_ready] = false;
            flag[intake_ready]=false;
            flag[pre_samp] = false;
            flag[pre_spec]=false;
            flag[spec] = false;

    }

    public void intake() {

        if(!timer(500,intake)) return;
        timer(0,intake);
//        Intake.setPower(1.0);
      //  pidfsetting(rotate_in0,pidf_intake_down);
        color_det = 0;
        while (Op.opModeIsActive() && color_det == 0 && !timer(1000,intake)){
            delay(25);
            flag[color_check] = color();

        }

        if (!flag[color_check]){
//            Intake.setPower(-0.7);
            delay(300);
//            Intake.setPower(0);
            pre_intake();
        }
        else{
//            Intake_handle.setPosition(handle_outtake);
//            Intake_rot.setPosition(handlerot_intake);
            pidfsetting(rotate_idle,pidf_intake_up);
            linearslide(slide_idle,slidev2);
            flag[intake_ready]=false;
            flag[button_flip]=false;
            // intake_level=0;// might drop
            delay(300);
//            Intake.setPower(0);


        }


    }
    public void sampleintake() {
       stop_drive();
       pidfsetting(arm_arngle_intake,pidf_intake);
       delay(300); // 500;
        Claw.setPosition(claw_close);
        delay(350);
       Left_handle.setPosition(lefthandle_idle);
       Right_handle.setPosition(righthandle_idle);
       delay(100);
       //pre_intakeidley
       linearslide(slide_idle,slidev2);
       flag[preintakeidle]=true;
       flag[intakeidleready]=false;
       timer(0,intake);
       }



    public boolean intake_specimen() {
        if (!flag[spec]){
//            Intake.setPower(0.4);
            move(-0.25);
            delay(300);
            stop_drive();
//            Intake.setPower(0);
            pidfsetting(rotate_spec_out,pidf_outtake_spec);//pre postion
            flag[spec] = true;
            step[specimen]=1;
            timer(0,spec);
            flag[pre_spec]=false;
            return false;
        }

        if (timer(1000,spec)&&step[specimen]==1){
            pidfsetting(rotate_spec_out,pidf_outtake_spec1);
//            Intake_handle.setPosition(handle_specimen_outtake); // change value for spec outtake
            step[specimen]=2;
            timer(0,spec);
            return false;

        }

        if (timer(500,spec)&&step[specimen]==2){
            linearslide(slide_spec_out-20,slidev2);
//            Intake_handle.setPosition(handle_specimen_outtake); // change value for spec outtake
            flag[spec] = false;
            flag[intake_ready]=false;
            step[specimen]=0;
            return true;
        }

        return false;


    }

    public void aspec_intake() {

                stop_drive();
                delay(200);
        //Intake_handle.setPosition(handle_idle); // change later
//            Intake_rot.setPosition(handlerot_intake);

           //pidfsetting(rotate_spec_in+300,pidf_intake_aspec);
             move(-0.25);
             delay(310);
//            Intake.setPower(0.4);
            move(-0.18);
            delay(300);
            stop_drive();
            pidfsetting(rotate_spec_out,pidf_outtake_spec);//pre postion.
             move(0.3);
//            Intake.setPower(0);
            delay(200);
//            Intake_handle.setPosition(handle_specimen_outtake);
           // stop_drive();


    }




    public void aspec_outtake() {

        move(0.3);
       pidfsetting(rotate_spec_out,pidf_outtake_spec1);
       delay(200);
        linearslide(slide_spec_out-65,slidev2);
//       Intake_handle.setPosition(handle_specimen_outtake);

        while (bar_dist.getDistance(DistanceUnit.MM)>105&& Op.opModeIsActive())
        {
            armrotatePIDF();
        }
//        Intake.setPower(0.9);
        delay(50);
//        Intake_handle.setPosition(handle_specimen_outtake - 0.04);
        linearslide(slide_spec_out-55,slidev2);
        delay(20);
        stop_drive();

        pidfsetting(rotate_spec_out + 600, pidf_outtake_spec_down);
        delay(300); //250
//        Intake.setPower(-0.8);
        pidfsetting(rotate_spec_out + 600, pidf_aspec_outtake);
        delay(150); //150
//        Intake.setPower(0);
        move(-0.6);
        if(!flag[last]) {
            linearslide(slide_idle-10, slidev2);
            delay(250);
//            Intake_handle.setPosition(handle_idle); // change later
            //  Intake_rot.setPosition(handlerot_intake);
            pidfsetting(rotate_spec_in+50, pidf_intake_spec);

            // linearslide(slide_idle, slidev2);

        }
        else {
            linearslide(slide_idle, slidev2);
//            Intake_handle.setPosition(handle_idle);
            delay(250);
//            Intake_handle.setPosition(handle_intake); // change later
            //  Intake_rot.setPosition(handlerot_intake);
            pidfsetting(200 , pidf_aspec_outtake);
            delay(100);
            linearslideTq(-10, 0.6);


        }

      //  Intake_handle.setPosition(handle_intake);
       // pidfsetting(rotate_spec_in +200,pidf_intake_spec);
    }


//    public void rbg(double power) {
//        leftFront.setPower(power);
//        rightFront.setPower(power);
//        leftBack.setPower(power);
//        rightBack.setPower(power);
//    }

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




    public void linearslideTq( int target, double power) {

        //if (target > lshi || target < lslo) return;
        Slide_top.setTargetPosition(target); // ne
        Slide_bot.setTargetPosition(target);
        Slide_top.setPower(power);
        Slide_bot.setPower(power);

    }

    public double  arm_angle_update()


    {
        arm_angle = 360 - ((Arm_encoder.getVoltage() / 3.2 * 360 + arm_angle_offset) % 360);
        if ( arm_angle > 300) arm_angle-=360;
        return arm_angle;

    }


    public void pidfsetting(double target, int index)

    {
        if(index<1) {

            p=0.0004;
            i=0;
            d=0;

        }

         else {

        p = pidftable[index][pp];
        i = pidftable[index][ii];
        d = pidftable[index][dd];
    }
        arm_angle_target=target;
        controller.setPID(p,i,d);
        armrotatePIDF();
    }





    public final void pause(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }




    public void resetmode(boolean ls)


    {
        Arm_left.setPower(0.2);
        Arm_right.setPower(-0.2);
        pause(1000);
        Arm_left.setPower(-0.14);
        Arm_right.setPower(0.14);
        pause(100);

      if(ls)  {
            Slide_bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slide_bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Slide_top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slide_top.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Arm_left.setPower(-0.13);
            Arm_right.setPower(0.13);
            delay(100);
            Slide_top.setPower(-0.35);
            Slide_bot.setPower(-0.35);
            pause(1000);
            Slide_top.setPower(-0.25);
            Slide_bot.setPower(-0.25);
            Arm_left.setPower(0);
            Arm_right.setPower(0);
            pause(1000);
            Slide_top.setPower(0);
            Slide_bot.setPower(0);
            Arm_left.setPower(0);
            Arm_right.setPower(0);
            pause(200);
            Slide_bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slide_bot.setTargetPosition(0);
            Slide_bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide_bot.setVelocity(0);

            Slide_top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slide_top.setTargetPosition(0);
            Slide_top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide_top.setVelocity(0);
            directdle();
        }
     //  else rotation_reset();



    }



    public void init(int tstep)//

    {
       if(tstep==0) { //for both

           imu.resetYaw();
           Gearbox.setPosition(0);
           controller = new PIDController(p, i, d);
           pause(300);
//           Intake_rot.setPosition(handlerot_intake);

           flag[drive] = true;

           pidftable[pidf_intake_up][pp]=0.0024;  pidftable[pidf_intake_up][ii]=0;  pidftable[pidf_intake_up][dd]=0.0001;
           pidftable[pidf_intake_idle][pp]=0.003;  pidftable[pidf_intake_idle][ii]=0;  pidftable[pidf_intake_idle][dd]=0.00008;
           pidftable[pidf_intake][pp]=0.002;  pidftable[pidf_intake][ii]=0;  pidftable[pidf_intake][dd]=0.00005;
           pidftable[pidf_outtake_up][pp]=0.00037;  pidftable[pidf_outtake_up][ii]=0;  pidftable[pidf_outtake_up][dd]=0.000023;//
           pidftable[pidf_outtake_up2][pp]=0.00075;  pidftable[pidf_outtake_up2][ii]=0.00012;  pidftable[pidf_outtake_up2][dd]=0.0001;//0.00075 // i = 0.00012
           pidftable[pidf_intake_spec][pp]=0.00035;  pidftable[pidf_intake_spec][ii]=0;  pidftable[pidf_intake_spec][dd]=0.00006; // turn from 0 degrees to 180, may need to decrease P and increase D
           pidftable[pidf_intake_spec2][pp]=0.001;  pidftable[pidf_intake_spec2][ii]=0.00012;  pidftable[pidf_intake_spec2][dd]=0.00002;
           pidftable[pidf_outtake_down][pp]=0.00035;  pidftable[pidf_outtake_down][ii]=0;  pidftable[pidf_outtake_down][dd]=0.00012;
           pidftable[pidf_hang_up][pp]=0.0016;  pidftable[pidf_hang_up][ii]=0;  pidftable[pidf_hang_up][dd]=0.0001;
           pidftable[pidf_hang2][pp]=0.002;  pidftable[pidf_hang2][ii]=0;  pidftable[pidf_hang2][dd]=0.0001; // p used 0.002
           pidftable[pidf_hang4][pp]=0.002;  pidftable[pidf_hang4][ii]=0;  pidftable[pidf_hang4][dd]=0.0002; // p used 0.002
           pidftable[pidf_hang3][pp]=0.002; pidftable[pidf_hang3][ii]=0;  pidftable[pidf_hang3][dd]=0.0000;
           pidftable[pidf_intake_aspec][pp]=0.002;  pidftable[pidf_intake_aspec][ii]=0;  pidftable[pidf_intake_aspec][dd]=0.00001; // turn from 0 degrees to 180, may need to decrease P and increase D

           pidftable[pidf_outtake_spec][pp]=0.00025;  pidftable[pidf_outtake_spec][ii]=0;  pidftable[pidf_outtake_spec][dd]=0.00002; // may need to decrease p in future (large turn)
           pidftable[pidf_outtake_spec1][pp]=0.0016;  pidftable[pidf_outtake_spec1][ii]=0.00001;  pidftable[pidf_outtake_spec1][dd]=0.0000; // s0.0016
           pidftable[pidf_outtake_spec_down][pp]=0.0075;  pidftable[pidf_outtake_spec_down][ii]=0;  pidftable[pidf_outtake_spec_down][dd]=0.00001;
           pidftable[pidf_aspec_outtake][pp]=0.001;  pidftable[pidf_aspec_outtake][ii]=0;  pidftable[pidf_aspec_outtake][dd]=0.00001;
           pidftable[pidf_outtake_aspec_down][pp]=0.005;  pidftable[pidf_outtake_aspec_down][ii]=0;  pidftable[pidf_outtake_aspec_down][dd]=0.00001;
           pidftable[pidf_aintake_down][pp]=0.0015;  pidftable[pidf_aintake_down][ii]=0;  pidftable[pidf_aintake_down][dd]=0.00018;// 0.0015,0.0001
         //  pidftable[pidf_aouttake_up2][pp]=0.001;  pidftable[pidf_aouttake_up2][ii]=0.00018;  pidftable[pidf_aouttake_up2][dd]=0.0000;//0


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
           afmoveconfig[2] [ydis]=-38;
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
           afmoveconfig[6] [xdis]=18;//16
           afmoveconfig[6] [ydis]=-48;
           afmoveconfig[6] [adis]=0;
           afmoveconfig[6] [time]=2000;

           //strafe for outtake
           afmoveconfig[7] [speedg]=0.02;
           afmoveconfig[7] [strafeg]=0.15;
           afmoveconfig[7] [turng]=0.02;
           afmoveconfig[7] [speedmax]=0.6;
           afmoveconfig[7] [strafemax]=0.7;
           afmoveconfig[7] [turnmax]=0.25;
           afmoveconfig[7] [xdis]=15;
           afmoveconfig[7] [ydis]=-1;
           afmoveconfig[7] [adis]=0;
           afmoveconfig[7] [time]=2000;

           //strafe for intake
           afmoveconfig[8] [speedg]=0.02;
           afmoveconfig[8] [strafeg]=0.15;
           afmoveconfig[8] [turng]=0.02;
           afmoveconfig[8] [speedmax]=0.5;
           afmoveconfig[8] [strafemax]=0.7;
           afmoveconfig[8] [turnmax]=0.25;
           afmoveconfig[8] [xdis]=13;
           afmoveconfig[8] [ydis]=-36;
           afmoveconfig[8] [adis]=0;
           afmoveconfig[8] [time]=2000;
           //strafe for outtake
           afmoveconfig[9] [speedg]=0.02;
           afmoveconfig[9] [strafeg]=0.15;
           afmoveconfig[9] [turng]=0.02;
           afmoveconfig[9] [speedmax]=0.5;
           afmoveconfig[9] [strafemax]=0.7;
           afmoveconfig[9] [turnmax]=0.25;
           afmoveconfig[9] [xdis]=15;
           afmoveconfig[9] [ydis]=2;
           afmoveconfig[9] [adis]=0;
           afmoveconfig[9] [time]=2000;

           //strafe for intake
           afmoveconfig[10] [speedg]=0.02;
           afmoveconfig[10] [strafeg]=0.15;
           afmoveconfig[10] [turng]=0.02;
           afmoveconfig[10] [speedmax]=0.5;
           afmoveconfig[10] [strafemax]=0.7;
           afmoveconfig[10] [turnmax]=0.2;
           afmoveconfig[10] [xdis]=13;
           afmoveconfig[10] [ydis]=-36;
           afmoveconfig[10] [adis]=0;
           afmoveconfig[10] [time]=2000;
           //strafe for outtake
           afmoveconfig[11] [speedg]=0.02;
           afmoveconfig[11] [strafeg]=0.15;
           afmoveconfig[11] [turng]=0.02;
           afmoveconfig[11] [speedmax]=0.5;
           afmoveconfig[11] [strafemax]=0.7;
           afmoveconfig[11] [turnmax]=0.25;
           afmoveconfig[11] [xdis]=15;
           afmoveconfig[11] [ydis]=4; //2
           afmoveconfig[11] [adis]=0;
           afmoveconfig[11] [time]=2000;
           //strafe for parking
           afmoveconfig[12] [speedg]=0.02;
           afmoveconfig[12] [strafeg]=0.15;
           afmoveconfig[12] [turng]=0.02;
           afmoveconfig[12] [speedmax]=0.5;
           afmoveconfig[12] [strafemax]=0.7;
           afmoveconfig[12] [turnmax]=0.2;
           afmoveconfig[12] [xdis]=10;
           afmoveconfig[12] [ydis]=-40;
           afmoveconfig[12] [adis]=0;
           afmoveconfig[12] [time]=2000;


           // auto samples  move configuaiton
            // preload sample
           asconfig[0] [speedg]=0.028;
           asconfig[0] [strafeg]=0.27;
           asconfig[0] [turng]=0.015;
           asconfig[0] [speedmax]=0.25;
           asconfig[0] [strafemax]=0.35;
           asconfig[0] [turnmax]=0.18;
           asconfig[0] [xdis]=10;
           asconfig[0] [ydis]=3;
           asconfig[0] [adis]=-45;
           asconfig[0] [time]=1500;

           // forward to first sample intake
           asconfig[1] [speedg]=0.028;
           asconfig[1] [strafeg]=0.27;
           asconfig[1] [turng]=0.015;
           asconfig[1] [speedmax]=0.25;
           asconfig[1] [strafemax]=0.35;
           asconfig[1] [turnmax]=0.18;
           asconfig[1] [xdis]=18.5;//21
           asconfig[1] [ydis]=6;
           asconfig[1] [adis]=0;
           asconfig[1] [time]=1500;
           // move to first sample outtake
           asconfig[2] [speedg]=0.028;
           asconfig[2] [strafeg]=0.27;
           asconfig[2] [turng]=0.015;
           asconfig[2] [speedmax]=0.25;
           asconfig[2] [strafemax]=0.35;
           asconfig[2] [turnmax]=0.18;
           asconfig[2] [xdis]=10;
           asconfig[2] [ydis]=3;
           asconfig[2] [adis]=-45;
           asconfig[2] [time]=1500;
           // forward to 2nd sample intake
           asconfig[3] [speedg]=0.028;
           asconfig[3] [strafeg]=0.25;
           asconfig[3] [turng]=0.015;
           asconfig[3] [speedmax]=0.3;
           asconfig[3] [strafemax]=0.35;
           asconfig[3] [turnmax]=0.18;
           asconfig[3] [xdis]=19.5;
           asconfig[3] [ydis]=17;
           asconfig[3] [adis]=0;
           asconfig[3] [time]=1500;
           //move to 2nd sample outtake
           asconfig[4] [speedg]=0.03;
           asconfig[4] [strafeg]=0.3;
           asconfig[4] [turng]=0.018;
           asconfig[4] [speedmax]=0.3;
           asconfig[4] [strafemax]=0.35;
           asconfig[4] [turnmax]=0.18;
           asconfig[4] [xdis]=9;
           asconfig[4] [ydis]=2;
           asconfig[4] [adis]=-45;
           asconfig[4] [time]=1500;
           // forward 3rd sample intake
           asconfig[5] [speedg]=0.03;
           asconfig[5] [strafeg]=0.2;//0.15
           asconfig[5] [turng]=0.018;
           asconfig[5] [speedmax]=0.3;
           asconfig[5] [strafemax]=0.4;
           asconfig[5] [turnmax]=0.2;
           asconfig[5] [xdis]=14;
           asconfig[5] [ydis]=19.5;
           asconfig[5] [adis]=20;
           asconfig[5] [time]=1500;
           // 3rd sample outake
           asconfig[6] [speedg]=0.03;
           asconfig[6] [strafeg]=0.3;
           asconfig[6] [turng]=0.03;
           asconfig[6] [speedmax]=0.4;
           asconfig[6] [strafemax]=0.3;
           asconfig[6] [turnmax]=0.2;
           asconfig[6] [xdis]=10;//16
           asconfig[6] [ydis]=5;
           asconfig[6] [adis]=-45;
           asconfig[6] [time]=1500;

           //strafe for outtake
           asconfig[7] [speedg]=0.02;
           asconfig[7] [strafeg]=0.15;
           asconfig[7] [turng]=0.02;
           asconfig[7] [speedmax]=0.6;
           asconfig[7] [strafemax]=0.7;
           asconfig[7] [turnmax]=0.25;
           asconfig[7] [xdis]=15;
           asconfig[7] [ydis]=0;
           asconfig[7] [adis]=0;
           asconfig[7] [time]=2000;

           //strafe for intake
           asconfig[8] [speedg]=0.02;
           asconfig[8] [strafeg]=0.15;
           asconfig[8] [turng]=0.02;
           asconfig[8] [speedmax]=0.5;
           asconfig[8] [strafemax]=0.7;
           asconfig[8] [turnmax]=0.25;
           asconfig[8] [xdis]=12;
           asconfig[8] [ydis]=-36;
           asconfig[8] [adis]=0;
           asconfig[8] [time]=1500;
           //strafe for outtake
           asconfig[9] [speedg]=0.02;
           asconfig[9] [strafeg]=0.15;
           asconfig[9] [turng]=0.02;
           asconfig[9] [speedmax]=0.5;
           asconfig[9] [strafemax]=0.7;
           asconfig[9] [turnmax]=0.25;
           asconfig[9] [xdis]=15;
           asconfig[9] [ydis]=4;
           asconfig[9] [adis]=0;
           asconfig[9] [time]=1500;
           return;

       }

       if(tstep==1) {//for telop
           //Handle.setPosition(handleIdle);
//           Claw.setPosition(clawIntake);
//           linearslide(armSlide,slide[idle],armslideV1);
//           rotatetargetPIDF(rotate[idle]);
           Arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
           Arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           Arm_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           Arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
           Arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           Arm_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           Slide_bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           Slide_bot.setTargetPosition(0);
           Slide_bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           Slide_bot.setVelocity(0);

           Slide_top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           Slide_top.setTargetPosition(0);
           Slide_top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           Slide_top.setVelocity(0);


           // Slide_bot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
//            Intake_handle.setPosition(0.15);
//            Intake_rot.setPosition(handlerot_intake);
            pidfsetting(rotate_spec_first+200,pidf_intake_spec);
            delay(400);
         //   pidfsetting(rotate_spec_first,pidf_intake_down);
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
//         Intake.setPower(0.8);
//         Intake_handle.setPosition(0.28);
         pidfsetting(rotate_spec_first-500,pidf_outtake_spec_down);
            delay(150); //250
//            Intake.setPower(-0.9);
            delay(150); //150
//            Intake.setPower(0);
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

    public void asample_outtake() {

//        pidfsetting(rotate_outtake, pidf_outtake_up);
//        Intake_handle.setPosition(handle_outtake);
//        Intake_rot.setPosition(handlerot_intake);
//        delay(300);

       // pidfsetting(rotate_outtake+50, pidf_aouttake_up2);
        linearslide(slide_outtake-20, slidev2);
        delay(500);
        move(-0.2);//-0.18
       // delay(40);
        timer(0, 4);
       while (basket_dist.getDistance(DistanceUnit.MM) > 230 && Op.opModeIsActive() && !timer(1300, 4)) {//target 340
      //  while ( Op.opModeIsActive() && !timer(100, 4)) {//target 340

                armrotatePIDF();
        }
        stop_drive();
       // delay(2000000);
//        Intake.setPower(-0.26);//-0.28
        delay(300);
//        Intake.setPower(0);
       // delay(100);
//        Intake_handle.setPosition(0.3);// lift the handle for a temp  higher locaiton.
        delay(50);
        move(0.25);
        delay(100);
        linearslide(400, slidev2);
        delay(200);
        stop_drive();
//        Intake_handle.setPosition(handle_intake);
        delay(200);
        if(!flag[last]){
        linearslide(400, slidev0);
        delay(200);
        pidfsetting(roatate_prein0 ,pidf_outtake_down);}
        else {
            linearslide(-10, slidev1);
            delay(200);
            pidfsetting(roatate_prein0-250 ,pidf_outtake_down);
        }


    }

    public void pedrosample_preouttake() {

//        pidfsetting(rotate_outtake, pidf_outtake_up);
//        Intake_handle.setPosition(handle_outtake);
//        Intake_rot.setPosition(handlerot_intake);
//        delay(300);

       // pidfsetting(rotate_outtake+50, pidf_aouttake_up2);
        linearslide(slide_outtake-20, slidev2);
        // delay(40);
        timer(0, 4);
        // delay(2000000);



    }

    public void pedrosample_outtake() {

//        Intake.setPower(-0.26);//-0.28
        delay(300);
//        Intake.setPower(0);
        // delay(100);
//        Intake_handle.setPosition(0.3);// lift the handle for a temp  higher locaiton.
        delay(50);
        move(0.25);
        delay(100);
        linearslide(400, slidev2);
        delay(200);
        stop_drive();
//        Intake_handle.setPosition(handle_intake);
        delay(200);
        linearslide(-10, slidev1);
        delay(200);
        pidfsetting(roatate_prein0-250 ,pidf_outtake_down);


    }

    public void asamplefirstmove()

    {
        move(0.4);
        delay(150);
        pidfsetting(rotate_outtake, pidf_outtake_up);
//        Intake_handle.setPosition(handle_outtake);
//        Intake_rot.setPosition(handlerot_intake);
//        Intake.setPower(1);
        delay(200);
//        Intake.setPower(0);
        linearslide(400,slidev1);
    }



    public  void asample_intake() {

        linearslide(800,slidev2);
//        if(flag[last])  {linearslide(1200,slidev2);delay (250);Intake_rot.setPosition(handlerot_intake+0.15);delay(0);}
        delay(350);
      //  delay(100000000);
      //  Intake.setPower(1);
      //  delay(25);
        //delay(10000000);

        pidfsetting(-40,pidf_aintake_down);
        delay(20);
//        Intake.setPower(1);
        delay(400); // 500
//        Intake_handle.setPosition(handle_outtake);
        pidfsetting(250, pidf_outtake_up);
        delay(100);
//        Intake.setPower(0);
        linearslide(400,slidev1);
        delay(200);
        if(flag[last]) delay(100);

//       Intake_rot.setPosition(handlerot_intake);
        pidfsetting(rotate_outtake, pidf_outtake_up);
        delay(100);


        // linearslide(armRotate, rotate, armrotateV2);
//        flag[ain]=true;
//        rotatetargetPIDF(rotate[ain1]);
//        armrotatePIDF();
//        delay(350);
//        Claw.setPosition(clawClose);
//        delay(300); // 300
//        Handle.setPosition(handleOut1);
//        rotatetargetPIDF(rotate[idle]);
//        delay(50);
//        //  amove(0);
        // delay(2000000000);

        //  linearslide(armRotate, armrotatePreout0, armrotateV1);

    }

    public void amove( int step) {


        double yaw,atar,gap=1.5;
        double x1,xtar,ytar,xgap,yrange,xrange,y1,a1;
        double ygap,agap;
        double SPEED_GAIN = asconfig[step][speedg]; // 0.02  //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        double STRAFE_GAIN = asconfig[step][strafeg]; //0.03  //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        double TURN_GAIN = asconfig[step][turng];  //0.015  //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        double MAX_AUTO_SPEED = asconfig[step][speedmax];;   //  Clip the approach speed to this max value (adjust for your robot)
        double MAX_AUTO_STRAFE = asconfig[step][strafemax];;   //  Clip the approach speed to this max value (adjust for your robot)
        double MAX_AUTO_TURN =asconfig[step][turnmax];;

        //  updatePoseEstimate();
//        x1=pose.position.x;
//        y1=pose.position.y;
        //     a1=imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
        xtar=asconfig[step][xdis];
        ytar=asconfig[step][ydis];
        atar=asconfig[step][adis];
        timer(0,2);
        while (Op.opModeIsActive() &&!timer(asconfig[step][time],2)) //{// todo &&!timer3(1200)
        {  armrotatePIDF();
            updatePoseEstimate();
            x1=pose.position.x;
            y1=pose.position.y;
            a1=imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
            xgap=xtar-x1;
            ygap=ytar-y1;
            agap=atar-a1;


            if (Math.abs(xgap)<gap&&Math.abs(ygap)<gap&&  Math.abs(agap)<5){xo=x1;yo=y1;ao=a1;break;}
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
//            Intake_handle.setPosition(0.13);
//            Intake_rot.setPosition(handlerot_intake);
            pidfsetting(rotate_spec_first,pidf_intake_spec);
            delay(400);
            pidfsetting(rotate_spec_first,pidf_aspec_outtake);
            delay(200);
            linearslide(580,slidev2);// first outtake

            while(Op.opModeIsActive()&& dis>215)
            {
                movestraight(0.26);
                armrotatePIDF();
                dis=bar_dist.getDistance(DistanceUnit.MM);
//                if(!flag[specimen]&& dis <360)
//                {Intake.setPower(0.9);flag[specimen]=true;}
            }
            flag[specimen]=false;
//            Intake.setPower(0.9);
            stop_drive();
            delay(500);
         //  delay(100000000);
           //// Intake.setPower(0.8);
//            Intake_handle.setPosition(0.20);
            pidfsetting(rotate_spec_first-400,pidf_outtake_aspec_down);
          //  pidfsetting(rotate_spec_first-400,pidf_outtake_spec_down);
            delay(300); //250
         //   pidfsetting(rotate_spec_first-300,pidf_outtake_spec_down);
//            Intake.setPower(-0.9);
            pidfsetting(rotate_spec_first-500,pidf_outtake_spec1);
            delay(50); //150
//            Intake.setPower(0);
            move(-0.4);
            delay(100);
            linearslide(slide_idle, slidev2);
            delay(250);
//            Intake_handle.setPosition(handle_idle); // change later
          //  Intake_rot.setPosition(handlerot_intake);
            pidfsetting(rotate_spec_in,pidf_intake_spec);
            stop_drive();
            delay(100);
            flag[specimen]=false;

        }

    }


    public void armrotatePIDF() {

        ;
        slidePos = Slide_top.getCurrentPosition();

        pid = controller.calculate( arm_angle_update(),arm_angle_target);
        ff = Math.cos(Math.toRadians(arm_angle)) * (f + k*slidePos) ;// target
        power = pid + ff;
        Arm_left.setPower(-power);
        Arm_right.setPower(power);// to be changed director.

    }





    public boolean lift() {
        if(!flag[lift]) {
           // stop_drive();
           pidfsetting(rotate_outtake+50,pidf_outtake_up);
            flag[lift]=true;
            timer(0,lift);
//            Intake_handle.setPosition(handle_outtake);
//            Intake_rot.setPosition(handlerot_intake);
            return false;
        }
        if(-(Arm_right.getCurrentPosition())>2000){
            stop_drive();
            pidfsetting(rotate_outtake+50,pidf_outtake_up2);
            linearslide(slide_outtake,slidev2);
            flag[lift]=false;
            return true;
        }

        return false;
    }


    public void sample_roate_outake() {


            pidfsetting(rotate_outtake+50,pidf_outtake_up);


    }




//    public void claw(boolean close,boolean check)
//
//    {
//        if(close)
//        {Intake.setPower(0.9); delay(1000);}
//
//        else {Intake.setPower(-0.2);delay (300);} // -0.2
//        Intake.setPower(0);
//
//
//    }





    public boolean outtake() {
        if(!flag[outtake]) {
            stop_drive();
//           Intake.setPower(-0.24);
           delay(250);
//           Intake.setPower(0);
            delay(150);
//            Intake_handle.setPosition(handlerot_intake);
            flag[drive] = false;
             delay(150);
            flag[outtake] = true;
            move(0.25);
            delay(100);
            timer(0,outtake);
            linearslide(slide_idle,slidev2);

            return false;
        }

        if  (Slide_bot.getCurrentPosition()<slide_rotate){
            flag[drive] = true;
            stop_drive();
            linearslide(slide_idle,slidev0);
            delay(200);
            pidfsetting(rotate_idle,pidf_outtake_down);
//            Intake_handle.setPosition(handle_idle+0.15);
            flag[outtake]= false;
            intake_level=0;
            flag[intake_ready]=false;
            stop_drive();
            return true;

        }



       return false;

    }

    public boolean outtake_spec() {
        if (!flag[outtake]){
            stop_drive();
            flag[drive] = false;
//            Intake.setPower(0.8);
            linearslide(slide_spec_out+15,slidev2);
//            Intake_handle.setPosition(handle_specimen_outtake - 0.04);
            pidfsetting(rotate_spec_out + 600,pidf_outtake_spec_down ); //650
            delay(300); //200
//            Intake.setPower(-0.9);
            stop_drive();
            delay(200); //150
//            Intake.setPower(0);
            move(-0.8);
            flag[outtake] = true;
            linearslide(slide_idle,slidev2);
            delay(250);

            flag[drive] = true;
            pidfsetting(rotate_idle,pidf_outtake_down);
//            Intake_handle.setPosition(handle_idle+0.15);
            flag[outtake] = false;
            return true;
        }
        return false;





    }

    public void outtake_spec_pre() {
            stop_drive();
            //Intake.setPower(0.8);
            pidfsetting(1700,pidf_outtake_up2 ); //1575
            delay(500);
            linearslide(1140,slidev1);
//            Intake_handle.setPosition(0.34);

    }


    public void outtake_spec_test(){
        stop_drive();
//        Intake.setPower(0.8);
        delay(50);
        linearslide(1140+900,slidev2);
        //Intake_handle.setPosition(0.34);
      //  pidfsetting(1575,pidf_outtake_down ); //650
        delay(700);
//        Intake.setPower(-0.8);
        delay(100);
        move(-0.6);
        delay(400);
        stop_drive();
//        Intake.setPower(0);
        linearslide(slide_idle,slidev1);
        delay(2000);
        pidfsetting(rotate_idle,pidf_outtake_spec);



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


