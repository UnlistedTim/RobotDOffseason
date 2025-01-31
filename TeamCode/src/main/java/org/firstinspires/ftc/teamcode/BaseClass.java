package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;
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
    double claw_close=0.46,claw_open=0.04;
    double arm_angle_target,arm_pose,arm_pose_target;
    double arm_angle_idle=-8,arm_angle_preintake=10,arm_arngle_intake=5,arm_angle_sampleouttake=105,arm_angle_specintake=207,arm_angle_specouttake=31;
    double arot_angle = 0;
    int aslide = 0;
    double lefthandle_idle=0.46,lefthandle_intake=0.18,lefthandle_left45=0.14,lefthandle_left90=0.08,lefthandle_right45=0.22,lefthandle_right90=0.28;
    double lefthandle_sampleouttake=0.64,lefthandle_specintake=0.61,lefthandle_specouttake=0.64,lefthandle_start=0.12;
    int intake_rotate_index=0;

    double righthandle_idle=0.54,righthandle_intake=0.82,righthandle_left45=0.78,righthandle_left90=0.72,righthandle_right45=0.86,righthandle_right90=0.92;
    double righthandle_sampleouttake=0.36,righthandle_specintake=0.77,righthandle_specouttake=0.36,righthandle_start=0.88;

    int slide_idle=200,slide_preintake=400,slide_sampleouttake=1800,slide_specintake=0,slide_specouttake=700,slide_intakemax=1250;

    int  slide_rotate=450,lslo=0,lshi=1900;
    Pose2d pp0=new Pose2d(0, 0, 0);




    PIDController controller;

    boolean hangflag = false;

    int slidePos;
    double pid ,power, ff;
    double p = 0.00004, i = 0, d = 0.0001 ,f = 0.12,k = 0.0001; //0.000035
    double handlePos = 0.05, handleStep = 0.05;

    public static boolean baseblue = false, baseright = true,baserest;

    double[][] afmoveconfig= new double[30][20];
    double[][] asconfig= new double[10][10];
    int speedg=0,strafeg=1,turng=2,speedmax=3,strafemax=4,turnmax=5,xdis=6,ydis=7,adis=8,time=9;
    int pidf_index=0;
    double[][] pidftable= new double[30][3];
    int pidf_intake_up=20,pidf_sampleintake=1,pidf_sampleouttake=2, pidf_spinintake=3,pidf_specouttake=4, pidf_idle=5,pidf_outtake_down=1,pidf_outtake_up=3, pidf_intake_idle = 4,
            pidf_hang_up = 5,  pidf_hang3 = 7, pidf_outtake_spec = 8,
            pidf_outtake_spec_down = 9, pidf_outtake_spec1 = 10 ,
            pidf_afspecouttake=11,
            pidf_outtake_up2 = 11, pidf_hang0=11,pidf_hang1=12,pidf_hang2=13,
            pidf_intake_spec = 12, pidf_intake_spec2 = 13,pidf_intake_aspec=14,pidf_aspec_outtake=15,pidf_outtake_aspec_down=16,pidf_aintake_down=17, pidf_hang4 = 18,pidf_specintake=20;
     int  pidf_idle_sampleout=21,pidf_sampleout_idle=22,pidf_idle_specin=23,pidf_specin_idle=24,pidf_specin_specout=25,pidf_specout_specin=26,pidf_specin_sampleout=27,pidf_sampleout_specin=28,pidf_specout_idle=29;

    int pp=0,ii=1,dd=2;
    double xo,yo,ao;

    int roatate_prein0=415, rotate_in0=-40, rotate_idle=30,rotate_outtake=2400, rotate_spec_in = 4100, rotate_spec_out=1500;//intake rotat could not over 70

    int slidev0=1000,slidev1=1500,slidev2=2700;

    boolean[] flag = new boolean[]{false,false,false,false, false,false,false,false,false, false, false, false,false,false,false,false, false,false, false,false,false, false, false,false,false, false,false, false, false, false,false,false,false, false, false,false, false, false, false, false, false,false, false};
    double[] stoptime = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0,0};
    int[] step = new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0};
    final int start = 0, claw_lock=1, intake = 2, lift = 3, resampleintake = 4, intake_shift = 5, button_flip = 6;
    final int intake_rotate= 7, hang_timer= 8, last = 9, drive = 10, stateready = 11, hang = 12, vb = 13,smooth_adj=14;
    final int force = 15,first=16, hang0 = 17,idle_ready=18,arot=19,preidle=20,idleready=21, spec = 22,specouttaketime=23, pre_samp = 24,presampleintake=25,sampleintakeready=26;
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







//    public boolean color () {
//        color_det = 0;
//        if (Intake_color.green() > 300){
//            color_det = 3; //detect yellow
//            return true; // take
//        }
//        if (Intake_color.red() > 150){
//            color_det = 1; //detect red
//            if (baseblue) return false; // if blue spit
//            else return true; // if red take
//        }
//        if (Intake_color.blue() > 120){
//            color_det = 2; //detect blue
//            if (baseblue) return true; // if blue take
//            else return false; // if red spit
//        }
//        return false;
//    }
    //forhang
    public void hang() {
        if(!flag[hang0] || Slide_top.getCurrentPosition() < 3000 ) return;

        move(0.35);
        linearslideTq(3300,0.98);
        pidfsetting(52); // Hit arm with low rung //1500

        while (arm_angle_update()<48 && Op.opModeIsActive())
        {delay(25);}
       delay(100);

        linearslideTq(2900,0.98);

        pidfsetting(90); //1600
        //  rbg.delay(1000);
        linearslideTq(-500,0.98);
        stop_drive();

        while(Op.opModeIsActive()&&Slide_top.getCurrentPosition() > -250) {delay(25);}


        pidfsetting(85);
        delay(50);
        linearslideTq(4800,0.98);
        while(Op.opModeIsActive() && Slide_top.getCurrentPosition() < 4400){delay(25);}
        pidf_index=pidf_hang1;
        pidfsetting(140); // 2700
        delay(800);
        linearslideTq(-600,0.98);
        while (Op.opModeIsActive() && Slide_top.getCurrentPosition() > 4350){delay(25);}
        delay(50);
        pidfsetting(60);
        linearslideTq(-600,1);



        while(Op.opModeIsActive() && Slide_top.getCurrentPosition() > -550){

            if (Slide_top.getCurrentPosition() < 700 && !hangflag){
                pidfsetting(90);
                hangflag = true;
            }
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
            if(step[hang]==3) return;
            linearslide(0,slidev2);
            flag[hang] = true;
            step[hang]=1;
            timer(0,hang_timer);
            return;
        }
        if ( (step[hang]==1&& (Slide_top.getCurrentPosition() < 15) || timer(2000,hang_timer)) ){
            delay(100);
            linearslideTq(0,0);
            Gearbox.setPosition(0.95);
            pidf_index=pidf_hang0;
            pidfsetting(20);
            timer(0,hang);
            step[hang]=2;
            return;
        }

        if(step[hang]==2&&timer(600,hang)) {
            linearslideTq(3350,0.98);

           flag[hang]=false;
            flag[hang0]=true;
            step[hang]=3;

        }
    }






    public void movestraight(double power) {

            moveRobot(power, 0 , -imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES))*0.015);
    }

    public boolean timer(double period, int i) {

        if (period == 0) {
            stoptime[i] = runtime.milliseconds();
            return false;
        }
        return runtime.milliseconds() - stoptime[i] > period;
    }






    public void  intake_smooth_shift(double sticky) {

        if(sticky<-0.5)
        {linearslide(slide_intakemax,slidev0-100) ;
            flag[smooth_adj]=true;
            return ;
        }

        if(sticky>0.5)  {
            linearslide(slide_preintake,slidev0-100);
            flag[smooth_adj]=true;
            return;
        }
        if(flag[smooth_adj]) {
            linearslide( slidePos-10,slidev0-200);
            if(slidePos>1100) pidfsetting(arm_angle_preintake-2);
            else if(slidePos>700) pidfsetting(arm_angle_preintake-1);
                    else  pidfsetting(arm_angle_preintake);
            flag[smooth_adj]=false;}
    }
  public void intake_claw_rotate(double stickx)

  {
      if(!timer(300,intake_rotate))return;

    if (stickx<-0.4&& intake_rotate_index>-2)
    {intake_rotate_index-=1;timer(0,intake_rotate);}
  else if (stickx>0.4&&intake_rotate_index<2 ) {intake_rotate_index+=1;timer(0,intake_rotate);}
  else return;

    if(intake_rotate_index==-2) {Left_handle.setPosition(lefthandle_left90);Right_handle.setPosition(righthandle_left90);return;}
    if(intake_rotate_index==-1) {Left_handle.setPosition(lefthandle_left45);Right_handle.setPosition(righthandle_left45);return;}
    if(intake_rotate_index==0) {Left_handle.setPosition(lefthandle_intake);Right_handle.setPosition(righthandle_intake);return;}
    if(intake_rotate_index==1) {Left_handle.setPosition(lefthandle_right45);Right_handle.setPosition(righthandle_right45);return;}
    if(intake_rotate_index==2) {Left_handle.setPosition(lefthandle_right90);Right_handle.setPosition(righthandle_right90);return;}

  }











    public boolean drop()

    {
      Left_handle.setPosition(lefthandle_idle);
      Right_handle.setPosition(righthandle_idle);
      Claw.setPosition(claw_open);
     delay(300);
     flag[claw_lock]=false;
    if(timer(2000,intake))  {
         pidf_index=pidf_sampleout_idle;
         pre_idle();
         return true;
     }
     pidf_index=pidf_sampleintake;
     pre_sampleintake();
     return false;
    }


    public void pre_idle()

    {

        Claw.setPosition(claw_open);
        Left_handle.setPosition(lefthandle_idle);
        Right_handle.setPosition(righthandle_idle);
        k=0.0001;
        linearslide(slide_idle, slidev2);
        flag[preidle]=true;
        flag[idleready]=false;
    }

    public void idle_ready()

    {
        if(flag[preidle]) {

            if(speed_index<1 && slidePos<800){ k = 0.0001; speed_index=1;}
            if ( slidePos < slide_rotate) {
                pidfsetting(arm_angle_idle);
                flag[preidle] = false;
                timer(0,stateready);

            }
            return;
        }

       if (Math.abs(arm_angle-arm_angle_idle)<30|| timer(1500,stateready))
       {
           flag[idleready]=true;
           pidf_index=pidf_idle;
           pidfsetting(arm_angle_idle);
       }

    }

    public void pre_sampleintake()

    {
        linearslide(slide_preintake,slidev2);
        flag[button_flip]=false;
        flag[presampleintake]=true;
        flag[sampleintakeready]=false;
        intake_rotate_index=0;

    }


    public void resampleintake()

    {
        flag[resampleintake]=true;
        flag[sampleintakeready]=false;
        flag[button_flip] = false;
        flag[presampleintake]=true;

    }

    public void sampleintake_ready(boolean pad2rbumperpress)

    {
        if(!flag[button_flip] && !pad2rbumperpress) flag[button_flip]=true;


        if(flag[presampleintake]) {
           //  if (slidePos < slide_rotate || flag[resampleintake]) {  //slide roataiton target
            if (slidePos < slide_rotate) {
            pidfsetting(arm_angle_preintake);
            flag[presampleintake] = false;
           // flag[resampleintake] = false;
            timer(0,stateready);

            }
             return;

        }

        if (Math.abs(arm_angle-arm_angle_preintake)<15 || timer(1500,stateready))
        {
            Claw.setPosition(claw_open);
            flag[claw_lock]=false;
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
        if(flag[prespecintake]) {

            if (slidePos < slide_rotate) {  //slide roataiton target
                pidfsetting(arm_angle_specintake);
                flag[prespecintake] = false;
                timer(0,stateready);

            }
            return;
        }

        if (Math.abs(arm_angle-arm_angle_specintake)<20||timer(1500,stateready))
        {

            Left_handle.setPosition(lefthandle_specintake);
            Right_handle.setPosition(righthandle_specintake);
           pidf_index=pidf_specintake;
            pidfsetting(arm_angle_specintake);
            flag[specintakeready]=true;
        }


    }



    public void specplacment(){
        stop_drive();
       pidf_index=pidf_specintake;
        if( flag[claw_lock]) {
            Claw.setPosition(claw_open);
            delay(100);
            flag[claw_lock]=false;
        }
        flag[placement]=false;
    }
    public void specintake() {

            stop_drive();
            move(-0.2);
            delay(250);
            stop_drive();
            Claw.setPosition(claw_close);
            delay(250);
            flag[claw_lock]=true;


        }


    public void pre_specouttake() {


        pidf_index=pidf_specin_specout;
        pidfsetting(arm_angle_specouttake);
        delay(150);
        move(0.25);
        delay(100);
        Left_handle.setPosition(lefthandle_specouttake-0.05);
        Right_handle.setPosition(righthandle_specouttake+0.05);
        stop_drive();
        flag[prespecouttake] = true;
        flag[specouttakeready] = false;
        timer(0,stateready);


    }

    public void specouttake_ready()

    {

        if(flag[prespecouttake]){
                if (Math.abs(arm_angle_update()-arm_angle_specouttake)<20||timer(1500,stateready) ){
                k = 0.0003;
                linearslide(slide_specouttake, slidev1+150);
                flag[prespecouttake] = false;
            }
            return;
        }

        if (Math.abs( slidePos-slide_specouttake)<100)
        {
             pidf_index=pidf_specouttake;
            pidfsetting(arm_angle_specouttake);
            flag[specouttakeready]=true;
        }


    }
    public void specouttake() {

      //  boolean handleflag = false;

        double dist = bar_dist.getDistance(DistanceUnit.MM);

      //  Claw.setPosition(claw_close);


        move(1.0);
        while(Op.opModeIsActive() && dist>190) {
            dist = bar_dist.getDistance(DistanceUnit.MM);
//            if (!handleflag && dist<400){
//                Left_handle.setPosition(lefthandle_specouttake);
//                Right_handle.setPosition(righthandle_specouttake);
//                handleflag = true;
//
//            }
            armrotatePIDF();
        }
        stop_drive();
        Claw.setPosition(claw_open);

        delay(150);
        flag[claw_lock]=false;
        move(-0.9);
        delay(300);
     pidf_index=pidf_specout_idle;
        stop_drive();

    }




    public void  intakeidle_ready()

    {
        if( slidePos <slide_rotate&&flag[preintakeidle]){  //slide roataiton target
            flag[preintakeidle]=false;
            flag[intakeidleready]=true;
            flag[lift]=false;
        }

    }

    public boolean pre_samplelift(boolean driver)

    {
        if(!flag[claw_lock]) return false;
        flag[lift]=false;
       pidf_index=pidf_idle_sampleout;
      pidfsetting(arm_angle_sampleouttake);
        flag[presamplelift]=true;
        flag[sampleliftready]=false;
        if(driver) flag[lift]=true;
        timer(0,stateready);

        return true;

    }


    public void samplelift_ready()

    {

        if(flag[presamplelift]&&(Math.abs(arm_angle-arm_angle_sampleouttake)<20)||  timer(1500,stateready)){

            flag[presamplelift]=false;
            flag[sampleliftready]=true;
        }

    }

    public void pre_sampleouttake()

    {

        k = 0.0001;

        linearslide(slide_sampleouttake,slidev2);
        pidf_index=pidf_sampleouttake;
        pidfsetting(arm_angle_sampleouttake );//high p to stable the positon
        Left_handle.setPosition(lefthandle_sampleouttake);
        Right_handle.setPosition(righthandle_sampleouttake);
        flag[presampleouttake]=true;
        flag[sampleouttakeready]=false;
        speed_index=0.3;
        flag[lift]=false;

    }


    public void sampleouttake_ready()

    {
        if( slidePos >(slide_sampleouttake-20)){
            flag[presampleintake]=false;
            flag[sampleouttakeready]=true;
        }


    }
    public  void sampleouttake() {

            stop_drive();
            Claw.setPosition(claw_open);
            delay(200);
            flag[claw_lock]=false;
            Left_handle.setPosition(lefthandle_idle);
            Right_handle.setPosition(righthandle_idle);
            delay(150);
            move(0.3);
            delay(100);
           // linearslide(slide_idle,slidev2);
            //flag[drive] = false;
            stop_drive();

    }











    public void sampleintake() {
       stop_drive();
       pidfsetting(arm_arngle_intake);
       delay(180); // 500;
        Claw.setPosition(claw_close);
        delay(350);
        flag[claw_lock]=true;
       Left_handle.setPosition(lefthandle_idle);
       Right_handle.setPosition(righthandle_idle);
       delay(100);
       //pre_intakeidley
       linearslide(slide_idle,slidev2);
       flag[preintakeidle]=true;
       flag[intakeidleready]=false;
       timer(0,intake);
       }




    public void aspec_intake() {

            move(-0.22);
            updatePoseEstimate();

            while(pose.position.x>9)
            {
                armrotatePIDF();
                updatePoseEstimate();
            }
            stop_drive();

            Claw.setPosition(claw_close);
            delay(200);



            pidf_index=pidf_specin_specout;
            pidfsetting(arm_angle_specouttake+10);
            delay(50);

            pidf_index=pidf_specouttake;
            arot_angle = arm_angle_specouttake;
            aslide = slide_specouttake;


            move(0.8);
            delay(50);

            Left_handle.setPosition(lefthandle_specouttake-0.05);
            Right_handle.setPosition(righthandle_specouttake+0.05);

            k = 0.0003;

            timer(0,arot);
    }




    public void aspec_outtake() {

        move(0.45);
        double dis=500;
        if(flag[first])
        {

            Left_handle.setPosition(lefthandle_specouttake-0.05);
            Right_handle.setPosition(righthandle_specouttake+0.05);
            pidf_index=  pidf_afspecouttake;
            pidfsetting(arm_angle_specouttake+6);
            linearslide(slide_specouttake,slidev2);
            flag[first]=false;
        }
//        else {
//            pidf_index = pidf_specouttake;
//            pidfsetting(arm_angle_specouttake);
//
//
//        }
//        delay(400);


     while(Op.opModeIsActive()&& dis>215)
        {
            //movestraight(0.4);
           // move(0.4);
            armrotatePIDF();
            dis=bar_dist.getDistance(DistanceUnit.MM);
        }

        stop_drive();
        Claw.setPosition(claw_open);
        //delay(25);
        linearslide(0,slidev2);
        move(-1.0);
        Left_handle.setPosition(lefthandle_idle);
        Right_handle.setPosition(righthandle_idle);

        delay(200);
      //  linearslide(0,slidev2);

        pidf_index=pidf_specout_specin;
        pidfsetting(arm_angle_specintake-10);

//        Left_handle.setPosition(lefthandle_specintake);
//        Right_handle.setPosition(righthandle_specintake);

        pidf_index=pidf_specintake;
        aslide = 0;
        arot_angle = arm_angle_specintake-3;

        stop_drive();

        k = 0.0001;




        if (flag[last]){
            pidf_index=pidf_sampleout_idle;
            // pidfsetting(arm_angle_idle+10);
            arot_angle =arm_angle_idle+10;
            Left_handle.setPosition(lefthandle_idle);
            Right_handle.setPosition(righthandle_idle);
            return;

        }
        Left_handle.setPosition(lefthandle_specintake);
        Right_handle.setPosition(righthandle_specintake);
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
        if ( arm_angle > 330) arm_angle-=360;
        return arm_angle;

    }


    public void pidfsetting(double target)

    {
        p = pidftable[pidf_index][pp];
        i = pidftable[pidf_index][ii];
        d = pidftable[pidf_index][dd];

        arm_angle_target=target;
        arm_pose_target=target*22.755556;
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





    public void init(int tstep)//

    {
       if(tstep==0) { //for both

           flag[drive] = true;

           imu.resetYaw();
           Gearbox.setPosition(0);
           controller = new PIDController(p, i, d);
           pause(500);
//           Intake_rot.setPosition(handlerot_intake);
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

           pidftable[pidf_intake_up][pp]=0.0024;  pidftable[pidf_intake_up][ii]=0;  pidftable[pidf_intake_up][dd]=0.0001;
           pidftable[pidf_intake_idle][pp]=0.003;  pidftable[pidf_intake_idle][ii]=0;  pidftable[pidf_intake_idle][dd]=0.00008;
          // pidftable[pidf_intake][pp]=0.002;  pidftable[pidf_intake][ii]=0;  pidftable[pidf_intake][dd]=0.00005;
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



           pidftable[pidf_idle_sampleout][pp]=0.0003;  pidftable[pidf_idle_sampleout][ii]=0;  pidftable[pidf_idle_sampleout][dd]=0;//
           pidftable[pidf_sampleout_idle][pp]=0.00027;  pidftable[pidf_sampleout_idle][ii]=0.00001;  pidftable[pidf_sampleout_idle][dd]=0;//0.00024
           pidftable[pidf_idle_specin][pp]=0.0002;  pidftable[pidf_idle_specin][ii]=0;  pidftable[pidf_idle_specin][dd]=0;//
           pidftable[pidf_specin_idle][pp]=0.00016;  pidftable[pidf_specin_idle][ii]=0;  pidftable[pidf_specin_idle][dd]=0;//
           pidftable[pidf_specin_specout][pp]=0.0001;  pidftable[pidf_specin_specout][ii]=0;  pidftable[pidf_specin_specout][dd]=0;//
           pidftable[pidf_sampleout_specin][pp]=0.00022;  pidftable[pidf_sampleout_specin][ii]=0;  pidftable[pidf_sampleout_specin][dd]=0;//
           pidftable[pidf_specin_sampleout][pp]=0.00025;  pidftable[pidf_specin_sampleout][ii]=0;  pidftable[pidf_specin_sampleout][dd]=0;//
           pidftable[ pidf_specout_idle][pp]=0.00016;  pidftable[pidf_specout_idle][ii]=0;  pidftable[pidf_specout_idle][dd]=0.0000;//

            pidftable[ pidf_specout_specin][pp] = 0.00025; pidftable[pidf_specout_specin][ii]=0.00001;  pidftable[pidf_specout_specin][dd]=0;

           pidftable[pidf_sampleintake][pp]=0.002;  pidftable[pidf_outtake_spec][ii]=0;  pidftable[pidf_outtake_spec][dd]=0.00001;
           pidftable[pidf_specintake][pp]=0.002;  pidftable[pidf_specintake][ii]=0.00015;  pidftable[pidf_specintake][dd]=0.00008;
           pidftable[pidf_specouttake][pp]=0.00095;  pidftable[pidf_specouttake][ii]=0.00002;  pidftable[pidf_specouttake][dd]=0.000;
           pidftable[pidf_sampleouttake][pp]=0.0008;  pidftable[pidf_sampleouttake][ii]=0;  pidftable[pidf_sampleouttake][dd]=0.000;
           pidftable[pidf_idle][pp]=0.00075;  pidftable[pidf_idle][ii]=0;  pidftable[pidf_idle][dd]=0.0001;
          // pidftable[pidf_afspinouttake][pp]=0.0003;  pidftable[pidf_afspinouttake][ii]=0;  pidftable[pidf_afspinouttake][dd]=0.;



           pidftable[pidf_hang0][pp]=0.0015;  pidftable[pidf_hang0][ii]=0;  pidftable[pidf_hang0][dd]=0.000;
           pidftable[pidf_hang1][pp]=0.003;  pidftable[pidf_hang1][ii]=0;  pidftable[pidf_hang1][dd]=0.00002;
           pidftable[pidf_afspecouttake][pp]=0.0012;  pidftable[pidf_afspecouttake][ii]=0.00002;  pidftable[pidf_afspecouttake][dd]=0.0001;









           // strafe for smaples
           afmoveconfig[0] [speedg]=0.035;
           afmoveconfig[0] [strafeg]=0.35;//0.4
           afmoveconfig[0] [turng]=0.018;
           afmoveconfig[0] [speedmax]=0.7;
           afmoveconfig[0] [strafemax]=0.85;
           afmoveconfig[0] [turnmax]=0.2;
           afmoveconfig[0] [xdis]=14;
           afmoveconfig[0] [ydis]=-35;
           afmoveconfig[0] [adis]=0;
           afmoveconfig[0] [time]=2000;

           // forward to move sample
           afmoveconfig[1] [speedg]=0.08;
           afmoveconfig[1] [strafeg]=0.2;
           afmoveconfig[1] [turng]=0.03;
           afmoveconfig[1] [speedmax]=0.9;
           afmoveconfig[1] [strafemax]=0.5;
           afmoveconfig[1] [turnmax]=0.2;
           afmoveconfig[1] [xdis]=43;
           afmoveconfig[1] [ydis]=-43;
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
           afmoveconfig[2] [ydis]=-49;
           afmoveconfig[2] [adis]=0;
           afmoveconfig[2] [time]=3000;
           // push first sample TO -6
           afmoveconfig[3] [speedg]=0.1;
           afmoveconfig[3] [strafeg]=0.2;
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
           afmoveconfig[4] [strafeg]=0.2;
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
           afmoveconfig[6] [speedg]=0.1;
           afmoveconfig[6] [strafeg]=0.2;
           afmoveconfig[6] [turng]=0.03;
           afmoveconfig[6] [speedmax]=0.90;
           afmoveconfig[6] [strafemax]=0.5;
           afmoveconfig[6] [turnmax]=0.2;
           afmoveconfig[6] [xdis]=19;//16
           afmoveconfig[6] [ydis]=-62;
           afmoveconfig[6] [adis]=0;
           afmoveconfig[6] [time]=2000;
           // back  for 3rd sample 44
           afmoveconfig[20] [speedg]=0.1;
           afmoveconfig[20] [strafeg]=0.2;
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
           afmoveconfig[21] [time]=2000;
           // push third sample
           afmoveconfig[22] [speedg]=0.04;
           afmoveconfig[22] [strafeg]=0.2;
           afmoveconfig[22] [turng]=0.03;
           afmoveconfig[22] [speedmax]=0.6;
           afmoveconfig[22] [strafemax]=0.5;
           afmoveconfig[22] [turnmax]=0.2;
           afmoveconfig[22] [xdis]=21;//16
           afmoveconfig[22] [ydis]=-68;
           afmoveconfig[22] [adis]=0;
           afmoveconfig[22] [time]=2000;



           //strafe for outtake
           afmoveconfig[7] [speedg]=0.02;
           afmoveconfig[7] [strafeg]=0.3;
           afmoveconfig[7] [turng]=0.02;
           afmoveconfig[7] [speedmax]=0.6;
           afmoveconfig[7] [strafemax]=0.9;
           afmoveconfig[7] [turnmax]=0.25;
           afmoveconfig[7] [xdis]=13;
           afmoveconfig[7] [ydis]=-6;
           afmoveconfig[7] [adis]=0;
           afmoveconfig[7] [time]=2000;

           //strafe for intake
           afmoveconfig[8] [speedg]=0.02;
           afmoveconfig[8] [strafeg]=0.2;
           afmoveconfig[8] [turng]=0.02;
           afmoveconfig[8] [speedmax]=0.7;
           afmoveconfig[8] [strafemax]=0.9;
           afmoveconfig[8] [turnmax]=0.25;
           afmoveconfig[8] [xdis]=13;
           afmoveconfig[8] [ydis]=-37;
           afmoveconfig[8] [adis]=0;
           afmoveconfig[8] [time]=2000;
           //strafe for outtake
           afmoveconfig[9] [speedg]=0.02;
           afmoveconfig[9] [strafeg]=0.2;
           afmoveconfig[9] [turng]=0.02;
           afmoveconfig[9] [speedmax]=0.7;
           afmoveconfig[9] [strafemax]=0.9;
           afmoveconfig[9] [turnmax]=0.25;
           afmoveconfig[9] [xdis]=15;
           afmoveconfig[9] [ydis]=-8;
           afmoveconfig[9] [adis]=0;
           afmoveconfig[9] [time]=2000;

           //strafe for intake
           afmoveconfig[10] [speedg]=0.02;
           afmoveconfig[10] [strafeg]=0.2;
           afmoveconfig[10] [turng]=0.02;
           afmoveconfig[10] [speedmax]=0.7;
           afmoveconfig[10] [strafemax]=0.9;
           afmoveconfig[10] [turnmax]=0.2;
           afmoveconfig[10] [xdis]=13;
           afmoveconfig[10] [ydis]=-37;
           afmoveconfig[10] [adis]=0;
           afmoveconfig[10] [time]=2000;
           //strafe for outtake
           afmoveconfig[11] [speedg]=0.02;
           afmoveconfig[11] [strafeg]=0.2;
           afmoveconfig[11] [turng]=0.02;
           afmoveconfig[11] [speedmax]=0.7;
           afmoveconfig[11] [strafemax]=0.9;
           afmoveconfig[11] [turnmax]=0.25;
           afmoveconfig[11] [xdis]=15;
           afmoveconfig[11] [ydis]=-10; //2
           afmoveconfig[11] [adis]=0;
           afmoveconfig[11] [time]=2000;

           afmoveconfig[12] [speedg]=0.02;
           afmoveconfig[12] [strafeg]=0.2;
           afmoveconfig[12] [turng]=0.02;
           afmoveconfig[12] [speedmax]=0.7;
           afmoveconfig[12] [strafemax]=0.9;
           afmoveconfig[12] [turnmax]=0.2;
           afmoveconfig[12] [xdis]=13;
           afmoveconfig[12] [ydis]=-37;
           afmoveconfig[12] [adis]=0;
           afmoveconfig[12] [time]=2000;
           //strafe for outtake
           afmoveconfig[13] [speedg]=0.02;
           afmoveconfig[13] [strafeg]=0.2;
           afmoveconfig[13] [turng]=0.02;
           afmoveconfig[13] [speedmax]=0.7;
           afmoveconfig[13] [strafemax]=0.9;
           afmoveconfig[13] [turnmax]=0.25;
           afmoveconfig[13] [xdis]=15;
           afmoveconfig[13] [ydis]=-12; //2
           afmoveconfig[13] [adis]=0;
           afmoveconfig[13] [time]=2000;
           //strafe for parking
           afmoveconfig[14] [speedg]=0.02;
           afmoveconfig[14] [strafeg]=0.2;
           afmoveconfig[14] [turng]=0.02;
           afmoveconfig[14] [speedmax]=0.7;
           afmoveconfig[14] [strafemax]=0.99;
           afmoveconfig[14] [turnmax]=0.2;
           afmoveconfig[14] [xdis]=8;
           afmoveconfig[14] [ydis]=-45;
           afmoveconfig[14] [adis]=0;
           afmoveconfig[14] [time]=2000;


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



        if(tstep==1) {// for auto size limitation
            flag[first]=true;
            Left_handle.setPosition(lefthandle_intake);
            Right_handle.setPosition(righthandle_intake);
            return;

        }


        if(tstep==2) {//first outake




        }



        if(tstep==5) {


            pidfsetting(rotate_spec_out);
            delay(500);
            linearslide(0,slidev0);
            delay(1000);
            pidfsetting(rotate_spec_in);
            delay(1000);





        }
    }

    public void afmove( int step, boolean str, boolean rot) {

        double yaw,atar;//,gap;
        boolean rot_flag = rot;
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
            if(str){
                if (rot_flag && timer(600,arot)){
                   // pidf_index=pidf_specintake;
                    pidfsetting(arot_angle);
                    linearslide(aslide,slidev1+150);
                    rot_flag = false;
                }

                if (Math.abs(ygap)<2) break;
            }
            else {if(Math.abs(xgap)<2) break;}
            xrange= Range.clip(xgap * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            yaw = Range.clip(agap * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            yrange= Range.clip(ygap * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            moveRobot(xrange, yrange , yaw);
        }
        stop_drive();

    }

    public void asample_outtake() {

//        pidfsetting(rotate_outtake, pidf_outtake_up);
//        Intake_handle.setPosition(handle_outtake);
//        Intake_rot.setPosition(handlerot_intake);
//        delay(300);

       // pidfsetting(rotate_outtake+50, pidf_aouttake_up2);
        linearslide(slide_sampleouttake-20, slidev2);
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
        pidfsetting(roatate_prein0 );}
        else {
            linearslide(-10, slidev1);
            delay(200);
            pidfsetting(roatate_prein0-250);
        }


    }

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

    public void specmove( ) {
        imu.resetYaw();
        pose=pp0;
        updatePoseEstimate();
        double x0=pose.position.x,y0=pose.position.y,a0=imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
        double xtar=x0+8,ytar=y0+35;
        double atar=a0,yrange,xrange,x1,y1,a1;
        double yaw;//,gap;


        double ygap=0,agap=0,xgap=0;
        double SPEED_GAIN = 0.03; // 0.02  //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        double STRAFE_GAIN = 0.3; //0.03  //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        double TURN_GAIN = 0.015;  //0.015  //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        double MAX_AUTO_SPEED = 0.6;   //  Clip the approach speed to this max value (adjust for your robot)
        double MAX_AUTO_STRAFE = 0.99;;   //  Clip the approach speed to this max value (adjust for your robot)
        double MAX_AUTO_TURN =0.2;;

        pidf_index=pidf_specin_specout;
        pidfsetting(arm_angle_specouttake+10);
        delay(50);
        move(0.5);
        delay(50);
        flag[prespecouttake] = true;
        flag[specouttakeready] = false;
        Left_handle.setPosition(lefthandle_specouttake-0.05);
        Right_handle.setPosition(righthandle_specouttake+0.05);


        timer(0,6);

        timer(0,specouttaketime);
        while (Op.opModeIsActive()&&!timer(2000,6)) {// todo &&!timer3(1200)
            armrotatePIDF();
            updatePoseEstimate();

            if(flag[prespecouttake]&& timer(600,specouttaketime)) {
                k=0.0003;
                pidf_index=pidf_specouttake;
                pidfsetting(arm_angle_specouttake+2);
                linearslide(slide_specouttake,slidev1+150);
                flag[prespecouttake]=false;
                flag[specouttakeready]=true;
            }
            
            x1=pose.position.x;
            y1=pose.position.y;
            a1=imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
            xgap=xtar-x1;
            ygap=ytar-y1;
            agap=atar-a1;

            if (Math.abs(ygap)<2) break;


            xrange= Range.clip(xgap * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            yaw = Range.clip(agap * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            yrange= Range.clip(ygap * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            moveRobot(xrange, yrange , yaw);
        }
        stop_drive();

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
        pidfsetting(roatate_prein0-250 );


    }

    public void asamplefirstmove()

    {
        move(0.4);
        delay(150);
        pidfsetting(rotate_outtake);
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

        pidfsetting(-40);
        delay(20);
//        Intake.setPower(1);
        delay(400); // 500
//        Intake_handle.setPosition(handle_outtake);
        pidfsetting(250);
        delay(100);
//        Intake.setPower(0);
        linearslide(400,slidev1);
        delay(200);
        if(flag[last]) delay(100);

//       Intake_rot.setPosition(handlerot_intake);
        pidfsetting(rotate_outtake);
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








    public void armrotatePIDF() {

        slidePos = Slide_top.getCurrentPosition();
        arm_pose= arm_angle_update()*22.75556;
        pid = controller.calculate( arm_pose,arm_pose_target);
        ff = Math.cos(Math.toRadians(arm_angle)) * (f + k *slidePos) ;
        power = pid + ff;
        Arm_left.setPower(-power);
        Arm_right.setPower(power);
    }


















    public void outtake_spec_pre() {
            stop_drive();
            //Intake.setPower(0.8);
            pidfsetting(1700); //1575
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
        pidfsetting(rotate_idle);

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


