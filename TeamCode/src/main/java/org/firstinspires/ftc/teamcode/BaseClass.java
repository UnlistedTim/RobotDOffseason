package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.arcrobotics.ftclib.util.InterpLUT;



public class BaseClass extends MecanumDrive {

    // Ticks from old rpm to new rpm for slide factor 0.4769124742


    public ElapsedTime runtime = new ElapsedTime();
    public LinearOpMode Op;
    public InterpLUT In_LHandle = new InterpLUT();
    public InterpLUT In_RHandle = new InterpLUT();

    public InterpLUT Out_LHandle = new InterpLUT();
    public InterpLUT Out_RHandle= new InterpLUT();

   // double intakerotpose=0,gearboxpose=0;

    // p 0.00012 down: spec outtake to intake idle (100) and sample outtake to intake idle (100)

    // p 0.0002 up: intake idle (100) to spec outtake and intake idle to sample outtake
    double speed_index=1;
    double drivinginput;
    public int courntnumber=0;
    public  double revmotencrate=28.407;
    public  int revpos=0,revtarget=0;
    public int deadband = 40;//20
    double arm_angle=0;
    double claw_close=0.42,claw_open=0.0;
    double arm_angle_target,arm_pose,arm_pose_target;
    double arm_angle_idle=-8,arm_angle_preintake=11,arm_arngle_intake=5,arm_angle_sampleouttake=105,arm_angle_specintake=208,arm_angle_specouttake=33; // 31
    double aarm_angle_specouttake =32;
    double arot_angle = 0;
    int aslide = 0;

    double cam_open =0.39, cam_close = 0.59;

    double intake_handle_intake = 1.0;

    double intake_arm_preintake = 0.58, intake_arm_intake = 0.62, intake_arm_intake_power = 0.7;

    double lefthandle_idle=0.47,lefthandle_intake=0.20,lefthandle_left45=0.14,lefthandle_left90=0.08,lefthandle_right45=0.24,lefthandle_right90=0.28;
    double lefthandle_sampleouttake=0.62,lefthandle_specintake=0.61,lefthandle_specouttake=0.67,lefthandle_start=0.12, lefthandle_fold = 0.04;
    int intake_rotate_index=0;  // old left spec intake 0.61, 0.77

    double righthandle_idle=0.53,righthandle_intake=0.84,righthandle_left45=0.78,righthandle_left90=0.72,righthandle_right45=0.88,righthandle_right90=0.92;
    double righthandle_sampleouttake=0.39,righthandle_specintake=0.79,righthandle_specouttake=0.35,righthandle_start=0.88, righthandle_fold = 0.98;

    int slide_idle=200,slide_preintake=400,slide_sampleouttake=1780,slide_specintake=0,slide_specouttake=710,slide_intakemax=1050;

    double curleft_handle = 0; double curright_handle = 0;

    // SPEC Intake Iterpolated Look up table for angle correction using handle servos to correct for arm angle error

    int  slide_rotate=450,lslo=0,lshi=1900;

    boolean deadzonecontrol = false, smoothshift = false;
    Pose2d pp0=new Pose2d(0, 0, 0);




    PIDController controller;
    PIDController lcontroller;


    boolean hangflag = false;
    boolean newlinearslides=false;

    int slidePos;
    double pid ,power, ff,lpid,lpower,lff, lkk;
    public double lp = 0.01, li = 0, ld = 0.0002,lf=0.05,lk=0.00018;//todo

    double p = 0.00004, i = 0, d = 0.0001 ,f = 0.12,k= 0.0001; //0.000035
    double handlePos = 0.05, handleStep = 0.05;



    public static boolean baseblue = false;//, baseright = true,baserest;

    double[][] afmoveconfig= new double[30][20];
    double[][] asconfig= new double[10][10];
    int speedg=0,strafeg=1,turng=2,speedmax=3,strafemax=4,turnmax=5,xdis=6,ydis=7,adis=8,time=9;
    int pidf_index=0;
    double[][] pidftable= new double[40][3];
    int pidf_intake_up=20,pidf_sampleintake=1,pidf_sampleouttake=2, pidf_spinintake=3,pidf_specouttake=4, pidf_idle=5,pidf_outtake_down=1,pidf_outtake_up=3, pidf_intake_idle = 4,
            pidf_hang_up = 5,  pidf_hang3 = 7, pidf_outtake_spec = 8,
            pidf_outtake_spec_down = 9, pidf_outtake_spec1 = 10 ,
            pidf_afspecouttake=11,
            pidf_outtake_up2 = 11, pidf_hang0=11,pidf_hang1=12,pidf_hang2=13,
            pidf_intake_spec = 12, pidf_intake_spec2 = 13,pidf_intake_aspec=14,pidf_aspec_outtake=15,pidf_outtake_aspec_down=16,pidf_aintake_down=17, pidf_hang4 = 18,pidf_specintake=20;
     int  pidf_idle_sampleout=21,pidf_sampleout_idle=22,pidf_idle_specin=23,pidf_specin_idle=24,pidf_specin_specout=25,pidf_specout_specin=26,pidf_specin_sampleout=27,pidf_sampleout_specin=28,pidf_specout_idle=29, pidf_aspecintake = 30, pidf_idle_demo = 31;

    int pp=0,ii=1,dd=2;
    double xo,yo,ao;

    int roatate_prein0=415, rotate_in0=-40, rotate_idle=30,rotate_outtake=2400, rotate_spec_in = 4100, rotate_spec_out=1500;//intake rotat could not over 70

    int slidev0=1000,slidev1=1500,slidev2=2700;

    boolean[] flag = new boolean[]{false,false,false,false,false,false, false,false,false,false,false, false, false, false,false,false,false,false, false,false, false,false,false, false, false,false,false, false,false, false, false, false,false,false,false, false, false,false, false, false, false, false, false,false, false};
    double[] stoptime = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0,0};
    int[] step = new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0};
    final int start = 0, claw_lock=1, intake = 2, lift = 3, resampleintake = 4, spec_adj = 5, button_flip = 6;
    final int intake_rotate= 7, hang_timer= 8, last = 9, drive = 10, stateready = 11, hang = 12, vb = 13,smooth_adj=14;
    final int force = 15,first=16, hang0 = 17,outspec=18,arot=19,preidle=20,idleready=21, specintake = 22,specouttaketime=23, pre_samp = 24,presampleintake=25,sampleintakeready=26;
    final int preintakeidle=27,intakeidleready=28,presampleouttake=29,sampleouttakeready=30,presamplelift=31,sampleliftready=32,prespecintake=33,specintakeready=34,placement=35,prespecouttake=36,specouttakeready=37;
    final  double arm_angle_offset=38;
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
        drivinginput= Math.abs(y) + Math.abs(x) + Math.abs(rx);
        double denominator = Math.max(drivinginput, 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower * ratio);
        rightFront.setPower(frontRightPower * ratio);
        leftBack.setPower(backLeftPower * ratio);
        rightBack.setPower(backRightPower * ratio);


    }

     public  void autocourtadjustment()
     {
         if(courntnumber==1){//first court red

             return;
         }
         if(courntnumber==2){//first court blue

             return;
         }
         if(courntnumber==3){// second court red

             return;
         }
         if(courntnumber==4){//second court blue

             return;
         }




//         afmoveconfig[0] [speedg]=0.02;
//         afmoveconfig[0] [strafeg]=0.35;//0.4
//         afmoveconfig[0] [turng]=0.018;
//         afmoveconfig[0] [speedmax]=0.6;
//         afmoveconfig[0] [strafemax]=0.85;
//         afmoveconfig[0] [turnmax]=0.2;
//         afmoveconfig[0] [xdis]=14;
//         afmoveconfig[0] [ydis]=-31;
//         afmoveconfig[0] [adis]=0;
//         afmoveconfig[0] [time]=2000;
//
//         // forward to move sample
//         afmoveconfig[1] [speedg]=0.08;
//         afmoveconfig[1] [strafeg]=0.18;
//         afmoveconfig[1] [turng]=0.03;
//         afmoveconfig[1] [speedmax]=0.9;
//         afmoveconfig[1] [strafemax]=0.5;
//         afmoveconfig[1] [turnmax]=0.2;
//         afmoveconfig[1] [xdis]=43;
//         afmoveconfig[1] [ydis]=-41;
//         afmoveconfig[1] [adis]=0;
//         afmoveconfig[1] [time]=2000;
//         // strafe for first sample 10
//         afmoveconfig[2] [speedg]=0.02;
//         afmoveconfig[2] [strafeg]=0.2;//0.2
//         afmoveconfig[2] [turng]=0.03;
//         afmoveconfig[2] [speedmax]=0.6;
//         afmoveconfig[2] [strafemax]=0.5;
//         afmoveconfig[2] [turnmax]=0.2;
//         afmoveconfig[2] [xdis]=42;
//         afmoveconfig[2] [ydis]=-48;
//         afmoveconfig[2] [adis]=0;
//         afmoveconfig[2] [time]=3000;
//         // push first sample TO -6
//         afmoveconfig[3] [speedg]=0.1;
//         afmoveconfig[3] [strafeg]=0.18;
//         afmoveconfig[3] [turng]=0.02;
//         afmoveconfig[3] [speedmax]=0.9;
//         afmoveconfig[3] [strafemax]=0.5;
//         afmoveconfig[3] [turnmax]=0.2;
//         afmoveconfig[3] [xdis]=18;
//         afmoveconfig[3] [ydis]=-52;
//         afmoveconfig[3] [adis]=0;
//         afmoveconfig[3] [time]=2000;
//         // back  for second sample 42
//         afmoveconfig[4] [speedg]=0.1;
//         afmoveconfig[4] [strafeg]=0.18;
//         afmoveconfig[4] [turng]=0.03;
//         afmoveconfig[4] [speedmax]=0.9;
//         afmoveconfig[4] [strafemax]=0.5;
//         afmoveconfig[4] [turnmax]=0.2;
//         afmoveconfig[4] [xdis]=42;
//         afmoveconfig[4] [ydis]=-51;
//         afmoveconfig[4] [adis]=0;
//         afmoveconfig[4] [time]=2000;
//         //strafe for second samples 8
//         afmoveconfig[5] [speedg]=0.04;
//         afmoveconfig[5] [strafeg]=0.2;//0.2
//         afmoveconfig[5] [turng]=0.03;
//         afmoveconfig[5] [speedmax]=0.95;
//         afmoveconfig[5] [strafemax]=0.5;
//         afmoveconfig[5] [turnmax]=0.2;
//         afmoveconfig[5] [xdis]=42;
//         afmoveconfig[5] [ydis]=-62;
//         afmoveconfig[5] [adis]=0;
//         afmoveconfig[5] [time]=2000;
//         // push second sample
//
//         afmoveconfig[6] [speedg]=0.11;//0.10
//         afmoveconfig[6] [strafeg]=0.2;
//         afmoveconfig[6] [turng]=0.03;
//         afmoveconfig[6] [speedmax]=0.92;
//         afmoveconfig[6] [strafemax]=0.5;
//         afmoveconfig[6] [turnmax]=0.2;
//         afmoveconfig[6] [xdis]=17;//16
//         afmoveconfig[6] [ydis]=-62;
//         afmoveconfig[6] [adis]=0;
//         afmoveconfig[6] [time]=2000;
//         // back  for 3rd sample 44
//         afmoveconfig[20] [speedg]=0.1;
//         afmoveconfig[20] [strafeg]=0.18;
//         afmoveconfig[20] [turng]=0.03;
//         afmoveconfig[20] [speedmax]=0.9;
//         afmoveconfig[20] [strafemax]=0.5;
//         afmoveconfig[20] [turnmax]=0.2;
//         afmoveconfig[20] [xdis]=44;
//         afmoveconfig[20] [ydis]=-62;
//         afmoveconfig[20] [adis]=0;
//         afmoveconfig[20] [time]=2000;
//         //strafe for third samples 8
//         afmoveconfig[21] [speedg]=0.04;
//         afmoveconfig[21] [strafeg]=0.2;//0.2
//         afmoveconfig[21] [turng]=0.025;
//         afmoveconfig[21] [speedmax]=0.5;
//         afmoveconfig[21] [strafemax]=0.5;
//         afmoveconfig[21] [turnmax]=0.2;
//         afmoveconfig[21] [xdis]=42;
//         afmoveconfig[21] [ydis]=-68.5;
//         afmoveconfig[21] [adis]=0;
//         afmoveconfig[21] [time]=2000;
//         // push third sample
//         afmoveconfig[22] [speedg]=0.05;
//         afmoveconfig[22] [strafeg]=0.16;//0.18
//         afmoveconfig[22] [turng]=0.03;
//         afmoveconfig[22] [speedmax]=0.7;
//         afmoveconfig[22] [strafemax]=0.4;
//         afmoveconfig[22] [turnmax]=0.2;
//         afmoveconfig[22] [xdis]=21.5;//16
//         afmoveconfig[22] [ydis]=-68.5;
//         afmoveconfig[22] [adis]=0;
//         afmoveconfig[22] [time]=2000;
//
//
//
//         //strafe for outtake
//         afmoveconfig[7] [speedg]=0.015;
//         afmoveconfig[7] [strafeg]=0.3;
//         afmoveconfig[7] [turng]=0.02;
//         afmoveconfig[7] [speedmax]=0.4;
//         afmoveconfig[7] [strafemax]=0.9;
//         afmoveconfig[7] [turnmax]=0.25;
//         afmoveconfig[7] [xdis]=12;
//         afmoveconfig[7] [ydis]=-6.5;
//         afmoveconfig[7] [adis]=0;
//         afmoveconfig[7] [time]=2000;
//
//         //strafe for intake
//         afmoveconfig[8] [speedg]=0.02;
//         afmoveconfig[8] [strafeg]=0.2;
//         afmoveconfig[8] [turng]=0.02;
//         afmoveconfig[8] [speedmax]=0.7;
//         afmoveconfig[8] [strafemax]=0.9;
//         afmoveconfig[8] [turnmax]=0.25;
//         afmoveconfig[8] [xdis]=14;
//         afmoveconfig[8] [ydis]=-40;
//         afmoveconfig[8] [adis]=0;
//         afmoveconfig[8] [time]=2000;
//         //strafe for outtake
//         afmoveconfig[9] [speedg]=0.02;
//         afmoveconfig[9] [strafeg]=0.2;
//         afmoveconfig[9] [turng]=0.02;
//         afmoveconfig[9] [speedmax]=0.7;
//         afmoveconfig[9] [strafemax]=0.9;
//         afmoveconfig[9] [turnmax]=0.25;
//         afmoveconfig[9] [xdis]=14;
//         afmoveconfig[9] [ydis]=-9;
//         afmoveconfig[9] [adis]=0;
//         afmoveconfig[9] [time]=2000;
//
//         //strafe for intake
//         afmoveconfig[10] [speedg]=0.02;
//         afmoveconfig[10] [strafeg]=0.2;
//         afmoveconfig[10] [turng]=0.02;
//         afmoveconfig[10] [speedmax]=0.7;
//         afmoveconfig[10] [strafemax]=0.9;
//         afmoveconfig[10] [turnmax]=0.2;
//         afmoveconfig[10] [xdis]=14;
//         afmoveconfig[10] [ydis]=-40;
//         afmoveconfig[10] [adis]=0;
//         afmoveconfig[10] [time]=2000;
//         //strafe for outtake
//         afmoveconfig[11] [speedg]=0.02;
//         afmoveconfig[11] [strafeg]=0.19;
//         afmoveconfig[11] [turng]=0.02;
//         afmoveconfig[11] [speedmax]=0.7;
//         afmoveconfig[11] [strafemax]=0.9;
//         afmoveconfig[11] [turnmax]=0.25;
//         afmoveconfig[11] [xdis]=14;
//         afmoveconfig[11] [ydis]=-11.5; //2
//         afmoveconfig[11] [adis]=0;
//         afmoveconfig[11] [time]=2000;
//         //strafe for intake
//
//         afmoveconfig[12] [speedg]=0.02;
//         afmoveconfig[12] [strafeg]=0.18;
//         afmoveconfig[12] [turng]=0.02;
//         afmoveconfig[12] [speedmax]=0.7;
//         afmoveconfig[12] [strafemax]=0.8;
//         afmoveconfig[12] [turnmax]=0.2;
//         afmoveconfig[12] [xdis]=14;
//         afmoveconfig[12] [ydis]=-41;// strafe gain lower
//         afmoveconfig[12] [adis]=0;
//         afmoveconfig[12] [time]=2000;
//         //strafe for outtake
//         afmoveconfig[13] [speedg]=0.02;
//         afmoveconfig[13] [strafeg]=0.17;
//         afmoveconfig[13] [turng]=0.02;
//         afmoveconfig[13] [speedmax]=0.6;
//         afmoveconfig[13] [strafemax]=0.8;
//         afmoveconfig[13] [turnmax]=0.25;
//         afmoveconfig[13] [xdis]=14;
//         afmoveconfig[13] [ydis]=-14; //2
//         afmoveconfig[13] [adis]=0;
//         afmoveconfig[13] [time]=2000;
//         //strafe for parking
//         afmoveconfig[14] [speedg]=0.03;
//         afmoveconfig[14] [strafeg]=0.3;
//         afmoveconfig[14] [turng]=0.02;
//         afmoveconfig[14] [speedmax]=0.7;
//         afmoveconfig[14] [strafemax]=0.99;
//         afmoveconfig[14] [turnmax]=0.2;
//         afmoveconfig[14] [xdis]=7;
//         afmoveconfig[14] [ydis]=-45;
//         afmoveconfig[14] [adis]=0;
//         afmoveconfig[14] [time]=2000;

     }

    //forhang
//    public void hang() {
//        if(!flag[hang0] || Slide_top.getCurrentPosition() < 3000 ) return;
//
//        move(0.35);
//        linearslideTq(3300,1.0);
//        pidfsetting(57); // Hit arm with low rung //1500
//
//        while (arm_angle_update()<46 && Op.opModeIsActive())
//        {delay(25);}
//       delay(150);
//
//        linearslideTq(2900,1.0);
//
//        pidfsetting(90); //1600
//        //  rbg.delay(1000);
//        linearslideTq(-500,1.0);
//        stop_drive();
//
//        while(Op.opModeIsActive()&&Slide_top.getCurrentPosition() > -250) {delay(25);}
//
//
//        pidfsetting(85);
//        delay(50);
//        linearslideTq(4800,1.0);
//        while(Op.opModeIsActive() && Slide_top.getCurrentPosition() < 4400){delay(25);}
//        pidf_index=pidf_hang1;
//        pidfsetting(140); // 2700
//        delay(800);
//        linearslideTq(-650,1.0);
//        while (Op.opModeIsActive() && Slide_top.getCurrentPosition() > 4350){delay(25);}
//        delay(50);
//        pidfsetting(60);
//        linearslideTq(-650,1);
//
//
//
//        while(Op.opModeIsActive() && Slide_top.getCurrentPosition() > -550){
//
//            if (Slide_top.getCurrentPosition() < 700 && !hangflag){
//                pidfsetting(90);
//                hangflag = true;
//            }
//            delay(25);
//        }
//        delay(2000);
//        linearslideTq(-650,0);
//        Arm_left.setPower(0);
//        Arm_right.setPower(0);
//        pause(100000);
//
//    }

//    public void hang2(boolean brea) {
//        if(!flag[hang0] || slidePos < 800 ) return;
//
//        move(0.35);
////        linearslideTq(3300,1.0);
//
//        pidf_index=pidf_hang0;
//
//        pidfsetting(57); // Hit arm with low rung //1500
//
//        while (arm_angle_update()<46 && Op.opModeIsActive())
//        {delay(25);}
//
//        delay(150);
//
//        Slide_top.setPower(-0.8);
//        Slide_bot.setPower(-0.8);
//
//
////        linearslideTq(2900,1.0);
//
//        pidfsetting(90); //1600
//        //  rbg.delay(1000);
////        linearslideTq(-500,1.0);
//        stop_drive();
//        delay(300);
//        Slide_top.setPower(-1.0);
//        Slide_bot.setPower(-1.0);
//        while(Op.opModeIsActive()&& slidePos > -70) {delay(25);}
//
//        delay(100);
//
//        Slide_top.setPower(0);
//        Slide_bot.setPower(0);
//
//
//
//
//        pidfsetting(85);
//        delay(100);
//
//        Slide_top.setPower(0.9);
//        Slide_bot.setPower(0.9);
//        delay(150);
//
//        while(Op.opModeIsActive() && slidePos < 1170){delay(25);}
//        pidf_index=pidf_hang1;
//        pidfsetting(140); // 2700
//        Slide_top.setPower(0);
//        Slide_bot.setPower(0);
//
//        delay(800);
//
//        Slide_top.setPower(-1.0);
//        Slide_bot.setPower(-1.0);
//
////        linearslideTq(-650,1.0);
//        while (Op.opModeIsActive() && slidePos > 1170){delay(25);}
//        delay(50);
//        pidfsetting(60);
////        linearslideTq(-650,1);
//
//
//
//        while(Op.opModeIsActive() && slidePos > -100 && !brea){
//
//
//            if (slidePos < 175 && !hangflag){
//                pidfsetting(90);
//                hangflag = true;
//            }
//            delay(25);
//        }
//        delay(300);
//        Slide_top.setPower(0);
//        Slide_bot.setPower(0);
////        linearslideTq(-650,0);
//        Arm_left.setPower(0);
//        Arm_right.setPower(0);
//        pause(100000);
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

//    public void pre_hang(){
//
//
//        if (!flag[hang]){
//
//            if(step[hang]==4) return;
//            linearslide(0,slidev2);
//            flag[hang] = true;
//            step[hang]=1;
//            timer(0,hang_timer);
//            delay(50);
//            return;
//        }
//
//        if ( (step[hang]==1&& (Slide_top.getCurrentPosition() < 15) || timer(1500,hang_timer)) ){
//            delay(50);
//
//            linearslide(0,0);
//            Gearbox.setPosition(0.95);
//            delay(100);
//             pidf_index=pidf_hang0;
//            pidfsetting(22);
//            timer(0,hang);
//            step[hang]=2;
//            return;
//        }
//
//        if(step[hang]==2&&timer(500,hang)) {
//
//            Left_handle.setPosition(lefthandle_fold);
//            Right_handle.setPosition(righthandle_fold);
//           linearslideTq(3350,0.98);
//          //  linearslide(3350,slidev2);
//
//            timer(0,hang);
//            step[hang]=3;
//
//        }
//
//        if (step[hang]==3 && timer(300,hang)){
//            Left_handle.setPosition(lefthandle_fold);
//            Right_handle.setPosition(righthandle_fold);
//            flag[hang]=false;
//            flag[hang0]=true;
//            step[hang]=4;
//        }
//    }

//    public void pre_hang2(){
//
//
//        if (!flag[hang]){
//
//            if(step[hang]==5) return;
//
//            pidfsetting(22);
//            linearslide(800,slidev2);
//            flag[hang] = true;
//            step[hang]=1;
//
//           // delay(50);
//            return;
//        }
//
//        if ( (step[hang]==1&& (slidePos > 730) ) ){
//
//                timer(0,hang_timer);
//                step[hang]=2;
//                Slide_top.setPower(0);
//                Slide_bot.setPower(0);
//                smoothshift = true;
//                return;
//            }
//
//
//        if(step[hang]==2&&timer(300,hang)) {
//
//            Gearbox.setPosition(0.95);
//            pidf_index = pidf_hang0;
//
//            timer(0, hang);
//            Left_handle.setPosition(lefthandle_fold);
//            Right_handle.setPosition(righthandle_fold);
//            step[hang] = 3;
//            return;
//        }
//
//
//         if(step[hang]==3&&timer(700,hang)) {
//
//             Slide_top.setPower(0.35);
//             Slide_bot.setPower(0.35);
//            timer(0,hang);
//            step[hang]=4;
//        }
//
//
//
//        if (step[hang]==4 && slidePos> 840 ){
//            Slide_top.setPower(0);
//            Slide_bot.setPower(0);
//            flag[hang]=false;
//            flag[hang0]=true;
//            step[hang]=5;
//        }
//    }
//





//    public void movestraight(double power) {
//
//            moveRobot(power, 0 , -imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES))*0.015);
//    }

    public boolean timer(double period, int i) {

        if (period == 0) {
            stoptime[i] = runtime.milliseconds();
            return false;
        }
        return runtime.milliseconds() - stoptime[i] > period;
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

        if(newlinearslides){
            smoothshift = false;
            revtarget=(int)(target*revmotencrate);
           if (target - slidePos  > 1000 ){
               lp = 0.013;
               ld= 0.0002;
           }
            else{
                lp = 0.008;
               ld= 0.00035;
           }
           lcontroller.setPID(lp, li, ld);

            if (deadzonecontrol){
                deadband = 15;
            }
            else{
                deadband = 40;
            }

        }
        else {
//            Slide_top.setTargetPosition(target); // ne
//            Slide_bot.setTargetPosition(target);
//            Slide_top.setVelocity(speed);
//            Slide_bot.setVelocity(speed);
        }

    }



    public final void pause(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }





    public void init(int tstep)//

    {
       if(tstep==0) { //for both

           backBotSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           backBotSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

           backTopSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           backTopSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

           frontBotSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           frontBotSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

           frontTopSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           frontTopSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

           leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
           leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
           rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
           rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

           backBotSlide.setDirection(DcMotorSimple.Direction.FORWARD);
           backTopSlide.setDirection(DcMotorSimple.Direction.REVERSE);
           frontBotSlide.setDirection(DcMotorSimple.Direction.FORWARD);
           frontTopSlide.setDirection(DcMotorSimple.Direction.REVERSE);

           imu.resetYaw();
//           Gearbox.setPosition(0);
//           controller = new PIDController(p, i, d);
           lcontroller = new PIDController(lp, li, ld);
           pause(100);


        // left 0.61, 0.77
         //  LHandle_correction.add(0,0.655);
           In_LHandle.add(197,lefthandle_specintake+0.04);
           In_LHandle.add(198,lefthandle_specintake+0.03);
           In_LHandle.add(200,lefthandle_specintake+0.015);
           In_LHandle.add(202,lefthandle_specintake);
           In_LHandle.add(206,lefthandle_specintake-0.02);
           In_LHandle.add(209,lefthandle_specintake-0.04);
          // LHandle_correction.add(350,0.575);
        //   LHandle_correction.add(350,0.4);


        // right 0.77

        //   RHandle_correction.add(0,0.725);
           In_RHandle.add(197,righthandle_specintake - 0.04);
           In_RHandle.add(198,righthandle_specintake - 0.03);
           In_RHandle.add(200,righthandle_specintake - 0.015);
           In_RHandle.add(202,righthandle_specintake);
           In_RHandle.add(206,righthandle_specintake + 0.02);
           In_RHandle.add(209,righthandle_specintake + 0.04);
        //   RHandle_correction.add(350,0.815);
      //     RHandle_correction.add(350,0.99);

           In_LHandle.createLUT();
           In_RHandle.createLUT();

        //   LHandle_outcorrect.add(0,0.665);
           Out_LHandle.add(27,0.66);
           Out_LHandle.add(29,0.62);
           Out_LHandle.add(32,0.59);
           Out_LHandle.add(35,0.585);
           Out_LHandle.add(36,0.575);
           Out_LHandle.add(38,0.555);
           Out_LHandle.add(39,0.545);
        //   LHandle_outcorrect.add(350,0.54);

//
//
//
        //   RHandle_outcorrect.add(0,0.335);
           Out_RHandle.add(27,0.34);
           Out_RHandle.add(29,0.38);
           Out_RHandle.add(32,0.41);
           Out_RHandle.add(35,0.415);
           Out_RHandle.add(36,0.425);
           Out_RHandle.add(38,0.445);
           Out_RHandle.add(39,0.455);
        //   RHandle_outcorrect.add(350,0.46);



           Out_LHandle.createLUT();
           Out_RHandle.createLUT();



           // strafe for smaples
           afmoveconfig[0] [speedg]=0.02;
           afmoveconfig[0] [strafeg]=0.35;//0.4
           afmoveconfig[0] [turng]=0.018;
           afmoveconfig[0] [speedmax]=0.6;
           afmoveconfig[0] [strafemax]=0.85;
           afmoveconfig[0] [turnmax]=0.2;
           afmoveconfig[0] [xdis]=14;
           afmoveconfig[0] [ydis]=-31;
           afmoveconfig[0] [adis]=0;
           afmoveconfig[0] [time]=2000;

           // forward to move sample
           afmoveconfig[1] [speedg]=0.08;
           afmoveconfig[1] [strafeg]=0.18;
           afmoveconfig[1] [turng]=0.03;
           afmoveconfig[1] [speedmax]=0.9;
           afmoveconfig[1] [strafemax]=0.5;
           afmoveconfig[1] [turnmax]=0.2;
           afmoveconfig[1] [xdis]=43;
           afmoveconfig[1] [ydis]=-41;
           afmoveconfig[1] [adis]=0;
           afmoveconfig[1] [time]=2000;
           // strafe for first sample 10
           afmoveconfig[2] [speedg]=0.02;
           afmoveconfig[2] [strafeg]=0.2;//0.2
           afmoveconfig[2] [turng]=0.03;
           afmoveconfig[2] [speedmax]=0.6;
           afmoveconfig[2] [strafemax]=0.5;
           afmoveconfig[2] [turnmax]=0.2;
           afmoveconfig[2] [xdis]=42;
           afmoveconfig[2] [ydis]=-48;
           afmoveconfig[2] [adis]=0;
           afmoveconfig[2] [time]=3000;
           // push first sample TO -6
           afmoveconfig[3] [speedg]=0.1;
           afmoveconfig[3] [strafeg]=0.18;
           afmoveconfig[3] [turng]=0.02;
           afmoveconfig[3] [speedmax]=0.9;
           afmoveconfig[3] [strafemax]=0.5;
           afmoveconfig[3] [turnmax]=0.2;
           afmoveconfig[3] [xdis]=18;
           afmoveconfig[3] [ydis]=-52;
           afmoveconfig[3] [adis]=0;
           afmoveconfig[3] [time]=2000;
           // back  for second sample 42
           afmoveconfig[4] [speedg]=0.1;
           afmoveconfig[4] [strafeg]=0.18;
           afmoveconfig[4] [turng]=0.03;
           afmoveconfig[4] [speedmax]=0.9;
           afmoveconfig[4] [strafemax]=0.5;
           afmoveconfig[4] [turnmax]=0.2;
           afmoveconfig[4] [xdis]=42;
           afmoveconfig[4] [ydis]=-51;
           afmoveconfig[4] [adis]=0;
           afmoveconfig[4] [time]=2000;
           //strafe for second samples 8
           afmoveconfig[5] [speedg]=0.04;
           afmoveconfig[5] [strafeg]=0.2;//0.2
           afmoveconfig[5] [turng]=0.03;
           afmoveconfig[5] [speedmax]=0.95;
           afmoveconfig[5] [strafemax]=0.5;
           afmoveconfig[5] [turnmax]=0.2;
           afmoveconfig[5] [xdis]=42;
           afmoveconfig[5] [ydis]=-62;
           afmoveconfig[5] [adis]=0;
           afmoveconfig[5] [time]=2000;
           // push second sample

           afmoveconfig[6] [speedg]=0.11;//0.10
           afmoveconfig[6] [strafeg]=0.2;
           afmoveconfig[6] [turng]=0.03;
           afmoveconfig[6] [speedmax]=0.92;
           afmoveconfig[6] [strafemax]=0.5;
           afmoveconfig[6] [turnmax]=0.2;
           afmoveconfig[6] [xdis]=17;//16
           afmoveconfig[6] [ydis]=-62;
           afmoveconfig[6] [adis]=0;
           afmoveconfig[6] [time]=2000;
           // back  for 3rd sample 44
           afmoveconfig[20] [speedg]=0.1;
           afmoveconfig[20] [strafeg]=0.18;
           afmoveconfig[20] [turng]=0.03;
           afmoveconfig[20] [speedmax]=0.9;
           afmoveconfig[20] [strafemax]=0.5;
           afmoveconfig[20] [turnmax]=0.2;
           afmoveconfig[20] [xdis]=44;
           afmoveconfig[20] [ydis]=-62;
           afmoveconfig[20] [adis]=0;
           afmoveconfig[20] [time]=2000;
           //strafe for third samples 8
           afmoveconfig[21] [speedg]=0.04;
           afmoveconfig[21] [strafeg]=0.2;//0.2
           afmoveconfig[21] [turng]=0.025;
           afmoveconfig[21] [speedmax]=0.5;
           afmoveconfig[21] [strafemax]=0.5;
           afmoveconfig[21] [turnmax]=0.2;
           afmoveconfig[21] [xdis]=42;
           afmoveconfig[21] [ydis]=-68;
           afmoveconfig[21] [adis]=0;
           afmoveconfig[21] [time]=1000;
           // push third sample
           afmoveconfig[22] [speedg]=0.06;
           afmoveconfig[22] [strafeg]=0.12;//0.18
           afmoveconfig[22] [turng]=0.03;
           afmoveconfig[22] [speedmax]=0.6;
           afmoveconfig[22] [strafemax]=0.3;
           afmoveconfig[22] [turnmax]=0.2;
           afmoveconfig[22] [xdis]=21.5;//16
           afmoveconfig[22] [ydis]=-68;
           afmoveconfig[22] [adis]=0;
           afmoveconfig[22] [time]=2000;



           //strafe for outtake
           afmoveconfig[7] [speedg]=0.015;
           afmoveconfig[7] [strafeg]=0.3;
           afmoveconfig[7] [turng]=0.02;
           afmoveconfig[7] [speedmax]=0.4;
           afmoveconfig[7] [strafemax]=0.9;
           afmoveconfig[7] [turnmax]=0.25;
           afmoveconfig[7] [xdis]=12;
           afmoveconfig[7] [ydis]=-7;
           afmoveconfig[7] [adis]=0;
           afmoveconfig[7] [time]=2000;

           //strafe for intake
           afmoveconfig[8] [speedg]=0.02;
           afmoveconfig[8] [strafeg]=0.2;
           afmoveconfig[8] [turng]=0.02;
           afmoveconfig[8] [speedmax]=0.7;
           afmoveconfig[8] [strafemax]=0.9;
           afmoveconfig[8] [turnmax]=0.25;
           afmoveconfig[8] [xdis]=14;
           afmoveconfig[8] [ydis]=-40;
           afmoveconfig[8] [adis]=0;
           afmoveconfig[8] [time]=2000;
           //strafe for outtake
           afmoveconfig[9] [speedg]=0.02;
           afmoveconfig[9] [strafeg]=0.2;
           afmoveconfig[9] [turng]=0.02;
           afmoveconfig[9] [speedmax]=0.7;
           afmoveconfig[9] [strafemax]=0.9;
           afmoveconfig[9] [turnmax]=0.25;
           afmoveconfig[9] [xdis]=14;
           afmoveconfig[9] [ydis]=-9;
           afmoveconfig[9] [adis]=0;
           afmoveconfig[9] [time]=2000;

           //strafe for intake
           afmoveconfig[10] [speedg]=0.02;
           afmoveconfig[10] [strafeg]=0.2;
           afmoveconfig[10] [turng]=0.02;
           afmoveconfig[10] [speedmax]=0.7;
           afmoveconfig[10] [strafemax]=0.9;
           afmoveconfig[10] [turnmax]=0.2;
           afmoveconfig[10] [xdis]=14;
           afmoveconfig[10] [ydis]=-40;
           afmoveconfig[10] [adis]=0;
           afmoveconfig[10] [time]=2000;
           //strafe for outtake
           afmoveconfig[11] [speedg]=0.02;
           afmoveconfig[11] [strafeg]=0.19;
           afmoveconfig[11] [turng]=0.02;
           afmoveconfig[11] [speedmax]=0.7;
           afmoveconfig[11] [strafemax]=0.9;
           afmoveconfig[11] [turnmax]=0.25;
           afmoveconfig[11] [xdis]=14;
           afmoveconfig[11] [ydis]=-11.5; //2
           afmoveconfig[11] [adis]=0;
           afmoveconfig[11] [time]=2000;
           //strafe for intake

           afmoveconfig[12] [speedg]=0.02;
           afmoveconfig[12] [strafeg]=0.18;
           afmoveconfig[12] [turng]=0.02;
           afmoveconfig[12] [speedmax]=0.7;
           afmoveconfig[12] [strafemax]=0.8;
           afmoveconfig[12] [turnmax]=0.2;
           afmoveconfig[12] [xdis]=14;
           afmoveconfig[12] [ydis]=-41;// strafe gain lower
           afmoveconfig[12] [adis]=0;
           afmoveconfig[12] [time]=2000;
           //strafe for outtake
           afmoveconfig[13] [speedg]=0.02;
           afmoveconfig[13] [strafeg]=0.17;
           afmoveconfig[13] [turng]=0.02;
           afmoveconfig[13] [speedmax]=0.6;
           afmoveconfig[13] [strafemax]=0.8;
           afmoveconfig[13] [turnmax]=0.25;
           afmoveconfig[13] [xdis]=14;
           afmoveconfig[13] [ydis]=-14; //2
           afmoveconfig[13] [adis]=0;
           afmoveconfig[13] [time]=2000;
           //strafe for parking
           afmoveconfig[14] [speedg]=0.03;
           afmoveconfig[14] [strafeg]=0.3;
           afmoveconfig[14] [turng]=0.02;
           afmoveconfig[14] [speedmax]=0.7;
           afmoveconfig[14] [strafemax]=0.99;
           afmoveconfig[14] [turnmax]=0.2;
           afmoveconfig[14] [xdis]=7;
           afmoveconfig[14] [ydis]=-45;
           afmoveconfig[14] [adis]=0;
           afmoveconfig[14] [time]=2000;


           // auto samples  move configuaiton
            // preload sample
           asconfig[0] [speedg]=0.03;
           asconfig[0] [strafeg]=0.3;
           asconfig[0] [turng]=0.018;
           asconfig[0] [speedmax]=0.25;
           asconfig[0] [strafemax]=0.35;
           asconfig[0] [turnmax]=0.18;
           asconfig[0] [xdis]=8.5;
           asconfig[0] [ydis]=2;//3
           asconfig[0] [adis]=-50;//-45
           asconfig[0] [time]=1500;

           // forward to first sample intake
           asconfig[1] [speedg]=0.028;
           asconfig[1] [strafeg]=0.27;
           asconfig[1] [turng]=0.015;
           asconfig[1] [speedmax]=0.25;
           asconfig[1] [strafemax]=0.35;
           asconfig[1] [turnmax]=0.18;
           asconfig[1] [xdis]=20.5;//21
           asconfig[1] [ydis]=5;
           asconfig[1] [adis]=0;
           asconfig[1] [time]=1500;
           // move to first sample outtake
           asconfig[2] [speedg]=0.03;
           asconfig[2] [strafeg]=0.3;
           asconfig[2] [turng]=0.015;
           asconfig[2] [speedmax]=0.25;
           asconfig[2] [strafemax]=0.30;
           asconfig[2] [turnmax]=0.18;
           asconfig[2] [xdis]=11;//10
           asconfig[2] [ydis]=3;//3
           asconfig[2] [adis]=-52;
           asconfig[2] [time]=1800;
           // forward to 2nd sample intake
           asconfig[3] [speedg]=0.03;
           asconfig[3] [strafeg]=0.3;
           asconfig[3] [turng]=0.015;
           asconfig[3] [speedmax]=0.3;
           asconfig[3] [strafemax]=0.3;
           asconfig[3] [turnmax]=0.18;
           asconfig[3] [xdis]=23;
           asconfig[3] [ydis]=15.5;
           asconfig[3] [adis]=0;
           asconfig[3] [time]=2200;
           //move to 2nd sample outtake
           asconfig[4] [speedg]=0.02;
           asconfig[4] [strafeg]=0.3;
           asconfig[4] [turng]=0.025;//0.018
           asconfig[4] [speedmax]=0.3;
           asconfig[4] [strafemax]=0.3;
           asconfig[4] [turnmax]=0.3;//0.18
           asconfig[4] [xdis]=13;//13
           asconfig[4] [ydis]=1.5;
           asconfig[4] [adis]=-40;//
           asconfig[4] [time]=2000;
           // forward 3rd sample intake
           asconfig[5] [speedg]=0.03;
           asconfig[5] [strafeg]=0.2;//0.15
           asconfig[5] [turng]=0.018;
           asconfig[5] [speedmax]=0.3;
           asconfig[5] [strafemax]=0.4;
           asconfig[5] [turnmax]=0.2;
           asconfig[5] [xdis]=16;
           asconfig[5] [ydis]=21;
           asconfig[5] [adis]=20;
           asconfig[5] [time]=3000;
           // 3rd sample outake
           asconfig[6] [speedg]=0.03;
           asconfig[6] [strafeg]=0.3;
           asconfig[6] [turng]=0.03;
           asconfig[6] [speedmax]=0.4;
           asconfig[6] [strafemax]=0.3;
           asconfig[6] [turnmax]=0.2;
           asconfig[6] [xdis]=10;//16
           asconfig[6] [ydis]=1;
           asconfig[6] [adis]=-39;
           asconfig[6] [time]=2000;

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



        if(tstep==1) {// for auto spec
            flag[first]=true;
//            Left_handle.setPosition(lefthandle_intake);
//            Right_handle.setPosition(righthandle_intake);
            return;
        }


        if(tstep==2) {//for teleop

            flag[drive]=true;
           // flag[first]=true;
            return;




        }


        if(tstep==3) {//for auto sample


            // flag[first]=true;

            pidf_index=pidf_idle;
            flag[first]=true;
//            Left_handle.setPosition(lefthandle_intake);
//            Right_handle.setPosition(righthandle_intake);
            return;

        }

        if(tstep==5) {


//            pidfsetting(rotate_spec_out);
            delay(500);
            linearslide(0,slidev0);
            delay(1000);
//            pidfsetting(rotate_spec_in);
            delay(1000);





        }
    }
//
//    public void afmove( int step, boolean str, boolean rot) {
//
//        double yaw,atar;//,gap;
//        boolean rot_flag = rot;
//        double x1,xtar,ytar,yrange,xrange,y1,a1;
//        double ygap,agap,xgap;
//        double SPEED_GAIN = afmoveconfig[step][speedg]; // 0.02  //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
//        double STRAFE_GAIN = afmoveconfig[step][strafeg]; //0.03  //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
//        double TURN_GAIN = afmoveconfig[step][turng];  //0.015  //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
//
//        double MAX_AUTO_SPEED = afmoveconfig[step][speedmax];;   //  Clip the approach speed to this max value (adjust for your robot)
//        double MAX_AUTO_STRAFE = afmoveconfig[step][strafemax];;   //  Clip the approach speed to this max value (adjust for your robot)
//        double MAX_AUTO_TURN =afmoveconfig[step][turnmax];;
//        xtar=afmoveconfig[step][xdis];
//        ytar=afmoveconfig[step][ydis];
//        atar=afmoveconfig[step][adis];
//        timer(0,6);
//        while (Op.opModeIsActive()&&!timer(afmoveconfig[step][time],6)) {// todo &&!timer3(1200)
//            armrotatePIDF();
//            updatePoseEstimate();
//            x1=pose.position.x;
//            y1=pose.position.y;
//            a1=imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
//            xgap=xtar-x1;
//            ygap=ytar-y1;
//            agap=atar-a1;
//            if(str){
//                if (rot_flag && timer(600,arot)){
//                   // pidf_index=pidf_specintake;
//                    pidfsetting(arot_angle);
//                    linearslide(aslide,slidev1+300);
//                    rot_flag = false;
//                }
//
//                if (Math.abs(ygap)<2) break;
//            }
//            else {if(Math.abs(xgap)<2) break;}
//            xrange= Range.clip(xgap * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//            yaw = Range.clip(agap * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
//            yrange= Range.clip(ygap * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//            moveRobot(xrange, yrange , yaw);
//        }
//        stop_drive();
//        if(flag[last]) {
//            pidf_index= pidf_sampleout_idle;
//            pidfsetting(arm_angle_idle);
//            Left_handle.setPosition(lefthandle_idle);
//            Right_handle.setPosition(righthandle_idle);
//
//        }
//
//    }
//




    public void pedrosample_preouttake() {

//        pidfsetting(rotate_outtake, pidf_outtake_up);
//        Intake_handle.setPosition(handle_outtake);
//        Intake_rot.setPosition(handlerot_intake);
//        delay(300);

       // pidfsetting(rotate_outtake+50, pidf_aouttake_up2);
        linearslide(slide_sampleouttake-20, slidev2);
        // delay(40);
        timer(0, 4);
        // delay(2000000);



    }

//    public void specmove( ) {
//
//
//         updatePoseEstimate();
//        double x0=pose.position.x,y0=pose.position.y;
//        double xtar=x0+4.5,ytar=y0+37,atar= imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
//        double yrange,xrange;
//        double yaw;//,gap;
//
//
//        double ygap,agap,xgap;
//        double SPEED_GAIN = 0.009; // 0.015  //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
//        double STRAFE_GAIN = 0.3; //0.03  //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
//        double TURN_GAIN = 0.015;  //0.015  //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
//
//        double MAX_AUTO_SPEED = 0.2;   //  Clip the approach speed to this max value (adjust for your robot) 0.45
//        double MAX_AUTO_STRAFE = 0.85;   //  Clip the approach speed to this max value (adjust for your robot)
//        double MAX_AUTO_TURN =0.2;;
//
//        pidf_index=pidf_specin_specout;
//        pidfsetting(arm_angle_specouttake+20);//10
//        delay(50);
//        move(0.5);
//        delay(50);
//        flag[prespecouttake] = true;
//        flag[specouttakeready] = false;
////        Left_handle.setPosition(lefthandle_specouttake-0.05);
////        Right_handle.setPosition(righthandle_specouttake+0.05);
//        timer(0,specouttaketime);
//        while (Op.opModeIsActive()&&!timer(2000,specouttaketime)) {// todo &&!timer3(1200)
//            armrotatePIDF();
//            if(flag[prespecouttake]&& timer(700,specouttaketime)) {
//                k=0.0003;
//                pidf_index=pidf_specouttake;
//                pidfsetting(arm_angle_specouttake+1);
//                linearslide(slide_specouttake,slidev1+100);
//                curleft_handle = lefthandle_specouttake-0.05;
//                curright_handle = righthandle_specouttake+0.05;
//                flag[prespecouttake]=false;
//                flag[specouttakeready]=true;
//            }
//            updatePoseEstimate();
//            xgap=xtar-pose.position.x;
//            ygap=ytar-pose.position.y;
//            agap=atar-imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
//
//            if (Math.abs(ygap)<2) break;
//
//
//            xrange= Range.clip(xgap * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//            yaw = Range.clip(agap * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
//            yrange= Range.clip(ygap * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//            moveRobot(xrange, yrange , yaw);
//        }
//        stop_drive();
//
//    }




    public void delay(double time)
    {

        int sleepcounter = (int) (time/25);
        for (int i = 0; i < sleepcounter; i++) {
            pause(20);
        }
    }




}


