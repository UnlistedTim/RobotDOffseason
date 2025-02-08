package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.security.KeyStore;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "PedroAutoSamples", group = "A")
@Config
public class PedroAutoSamples extends OpMode  {

    public double[][] ptable = new double[40][3];

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
  //  Pose2d pp = new Pose2d(0, 0, 0);

    int pp=0,ii=1,dd=2;

    BaseClass rbg;
    public OpMode op;


    double pid ,power, ff;

    int rotatePos,slidePos;

    public  DcMotorEx Arm_right, Arm_left, Slide_top,Slide_bot;

    public VoltageSensor voltageSensor;

    public  DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public  Servo Left_handle,Right_handle, Claw, Gearbox;
    public AnalogInput Arm_encoder;
    public ColorSensor Claw_color;
    public DistanceSensor bar_dist;
    public DistanceSensor basket_dist;
    public IMU imu;
    public ElapsedTime runtime = new ElapsedTime();
    int sampleouttake=1, sampleintake=2,idle=3;



    double claw_close=0.46,claw_open=0.04;
    double arm_angle_target,arm_pose,arm_pose_target;
    double arm_angle_idle=-8,arm_angle_preintake=10,arm_arngle_intake=5,arm_angle_sampleouttake=105,arm_angle_specintake=208,arm_angle_specouttake=31;
    double aarm_angle_specouttake =32;
    double arot_angle = 0;
    final  double arm_angle_offset=38;
    double arm_angle;
    int aslide = 0,pidf_index ,step=0;
    double lefthandle_idle=0.46,lefthandle_intake=0.18,lefthandle_left45=0.14,lefthandle_left90=0.08,lefthandle_right45=0.22,lefthandle_right90=0.28;
    double lefthandle_sampleouttake=0.64,lefthandle_specintake=0.61,lefthandle_specouttake=0.64,lefthandle_start=0.12;
    int intake_rotate_index=0;

    double righthandle_idle=0.54,righthandle_intake=0.82,righthandle_left45=0.78,righthandle_left90=0.72,righthandle_right45=0.86,righthandle_right90=0.92;
    double righthandle_sampleouttake=0.36,righthandle_specintake=0.77,righthandle_specouttake=0.36,righthandle_start=0.88;

    int slide_idle=200,slide_preintake=400,slide_sampleouttake=1800,slide_specintake=0,slide_specouttake=700,slide_intakemax=1250;

    int rotateTarget=0;



    PIDController controller;

    final double ticks_in_degree = 8192/360;

    public LED rear_led_red;
    public LED rear_led_green;

    double rotateStartangle=0 ;

    double p = 0.0000, i = 0, d = 0.000 ,f = 0.12,k = 0.000035; //0.000035
    final int force = 15,first=16, hang0 = 17,color_check=18,arot=19,preidle=20,idleready=21, spec = 22,specouttaketime=23, pre_samp = 24,presampleintake=25,sampleintakeready=26;
    final int preintakeidle=27,intakeidleready=28,presampleouttake=29,sampleouttakeready=30,presamplelift=31,sampleliftready=32,prespecintake=33,specintakeready=34,placement=35,prespecouttake=36,specouttakeready=37;

    int   idle_sampleout=21,sampleout_idle=22;
    double[] stoptime = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0,0};




    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(8.5, 3, Math.toRadians(-45)); // 9, 132

    private final Pose scoreControlPose = new Pose(21,121,Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park,scorePickup1;
    private PathChain scordrop1,Pickup1, grabPickup2, grabPickup3, scorePickup2, scorePickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose),  new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePickup1 = new Path(new BezierLine(new Point(scorePose),  new Point(pickup1Pose)));
        scorePickup1.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading());
        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scordrop1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
//                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */


        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */


        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    public void linearslide( int target, int speed) {
        Slide_top.setTargetPosition(target); // ne
        Slide_bot.setTargetPosition(target);
        Slide_top.setVelocity(speed);
        Slide_bot.setVelocity(speed);

    }



    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                   if(step==0) {
                       follower.followPath(scorePreload, true);
                       step++;

                   }
//                   if(pouttake()) {
//                       setPathState(1);
//                       break;
//                   }
                setPathState(1);




            case 1:
                    delay(100000000);
                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    /* Score Preload */



                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (Slide_top.getCurrentPosition() < 600 && -Arm_right.getCurrentPosition() < 600){
                        //follower.followPath(grabPickup1,true);
                        setPathState(2);
                        return;
                    }

                }
                break;
//            case 2:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
//                if(follower.getPose().getX() > (pickup1Pose.getX() - 1) && follower.getPose().getY() > (pickup1Pose.getY() - 1)) {
//                    /* Grab Sample */
//                    claw.groundClaw();
//                    claw.closeClaw();
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scorePickup1,true);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
//                    /* Score Sample */
//                    claw.scoringClaw();
//                    claw.openClaw();
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup2,true);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
//                if(follower.getPose().getX() > (pickup2Pose.getX() - 1) && follower.getPose().getY() > (pickup2Pose.getY() - 1)) {
//                    /* Grab Sample */
//                    claw.groundClaw();
//                    claw.closeClaw();
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scorePickup2,true);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
//                    /* Score Sample */
//                    claw.scoringClaw();
//                    claw.openClaw();
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup3,true);
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
//                if(follower.getPose().getX() > (pickup3Pose.getX() - 1) && follower.getPose().getY() > (pickup3Pose.getY() - 1)) {
//                    /* Grab Sample */
//                    claw.groundClaw();
//                    claw.closeClaw();
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scorePickup3, true);
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
//                    /* Score Sample */
//                    claw.scoringClaw();
//                    claw.openClaw();
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
//                    follower.followPath(park,true);
//                    setPathState(8);
//                }
//                break;
            case 8:

//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(follower.getPose().getX() > (parkPose.getX() - 1) && follower.getPose().getY() > (parkPose.getY() - 1)) {
//                    /* Put the claw in position to get a level 1 ascent */
//                    claw.startClaw();
//                    claw.closeClaw();
//
//                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    break;


        }
    }

    public final void pause(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void delay(double time)
    {

        int sleepcounter = (int) (time/25);
        for (int i = 0; i < sleepcounter; i++) {
            armrotatePIDF();
            pause(20);
        }
    }

//    public void pidfsetting(int target, int index)
//
//    {
//        p=ptable[index][pp];
//        i=ptable[index][ii];
//        d=ptable[index][dd];
//        rotateTarget=target;
//        armrotatePIDF();
//    }

//    public void armrotatePIDF()
//
//    {
//        rotatePos = -Arm_right.getCurrentPosition();
//        slidePos = Slide_top.getCurrentPosition();
//        controller.setPID(p,i,d);
//        pid = controller.calculate(rotatePos,rotateTarget);
//        ff = Math.cos(Math.toRadians(rotatePos/ticks_in_degree +rotateStartangle)) * (f + k*slidePos) ;// target
//        power = pid + ff;
//
////        if (power > 1.0) power = 0.98;
////        if (power < -1.0) power = -0.98;
//        Arm_left.setPower(-power);
//        Arm_right.setPower(power);// to be changed director.
//
//    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {



        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();


        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {

        IniHardware(hardwareMap);
        follower = new Follower(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        controller = new PIDController(p, i, d);
        follower.setStartingPose(startPose);
        buildPaths();
        telemetry.update();
        pause(500);
        telemetry.addLine("Ready for right bumper");
        telemetry.update();




        ptable[idle_sampleout][pp]=0.0003;  ptable[idle_sampleout][ii]=0;  ptable[idle_sampleout][dd]=0;//
        ptable[sampleout_idle][pp]=0.00027;  ptable[sampleout_idle][ii]=0.00001;  ptable[sampleout_idle][dd]=0;//0.00024


        ptable[sampleintake][pp]=0.002;  ptable[sampleintake][ii]=0;  ptable[sampleintake][dd]=0.000013;

        ptable[sampleouttake][pp]=0.0008;  ptable[sampleouttake][ii]=0;  ptable[sampleouttake][dd]=0.000;
        ptable[idle][pp]=0.00075;  ptable[idle][ii]=0;  ptable[idle][dd]=0.0001;


    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {

        telemetry.addData("Arm Pos", -Arm_right.getCurrentPosition());

        telemetry.addData("rotate targ",rotateTarget);

        telemetry.addData("arm power",power);

        telemetry.update();


        if (gamepad1.right_bumper){


            Claw.setPosition(claw_close);
            telemetry.addLine("Done");
            telemetry.update();

        }

   // armrotatePIDF();

    }

    public boolean pouttake() {
       if(step==1) {

           pidf_index = idle_sampleout;
           psetting(arm_angle_sampleouttake);
           step++;

       }

      if( arm_angle>arm_angle_sampleouttake-20&& step==2) {
          linearslide(slide_sampleouttake, 2500);
          pidf_index = sampleouttake;
          psetting(arm_angle_sampleouttake);
          Left_handle.setPosition(lefthandle_sampleouttake);
          Right_handle.setPosition(righthandle_sampleouttake);
       //   timer(0,1);
          step++;
      }

      if(Slide_top.getCurrentPosition()>(slide_sampleouttake-100)&&step==3){

          move(-0.24);//-0.18
          delay(50);
           step++;
      }

       if (basket_dist.getDistance(DistanceUnit.MM) < 180&&step==4 ) {//target 340// todo&& !timer(2000, 4

          // delay(50);
           step++;
        }

       if(step==5) {
           stop_drive();
           Claw.setPosition(claw_open);
           delay(150);
           Left_handle.setPosition(lefthandle_idle);
           Right_handle.setPosition(righthandle_idle);
           delay(50);
           move(0.4);
           delay(150);
           linearslide(slide_preintake, 2500);
           delay(100);
           stop_drive();
           delay(100);
           step=0;
           return true;
       }

       delay(25);
       return false;
    }


    public void preouttake() {






    }


    public void move(double power) {
        leftFront .setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }


    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }


    public void armrotatePIDF() {
        arm_angle_update();
        slidePos = Slide_top.getCurrentPosition();
        arm_pose= arm_angle*22.75556;
        pid = controller.calculate( arm_pose,arm_pose_target);
        ff = Math.cos(Math.toRadians(arm_angle)) * (f + k *slidePos) ;
        power = pid + ff;
        Arm_left.setPower(-power);
        Arm_right.setPower(power);
    }
    public double  arm_angle_update()


    {
        arm_angle = 360 - ((Arm_encoder.getVoltage() / 3.2 * 360 + arm_angle_offset) % 360);
        if ( arm_angle > 330) arm_angle-=360;
        return arm_angle;

    }

    public void stop_drive() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }


    public void psetting(double target)

    {
        p = ptable[pidf_index][pp];
        i = ptable[pidf_index][ii];
        d = ptable[pidf_index][dd];

        arm_angle_target=target;
        arm_pose_target=target*22.755556;
        controller.setPID(p,i,d);
        armrotatePIDF();
    }



    public boolean timer(double period, int i) {

        if (period == 0) {
            stoptime[i] = runtime.milliseconds();
            return false;
        }
        return runtime.milliseconds() - stoptime[i] > period;
    }
    public void IniHardware(HardwareMap hardwareMap) {


        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);


//        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
//        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
//                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;


        Arm_right = hardwareMap.get(DcMotorEx.class, "Arm_right");
        Arm_left = hardwareMap.get(DcMotorEx.class, "Arm_left");
        Slide_bot = hardwareMap.get(DcMotorEx.class, "Slide_bot");
        Slide_top = hardwareMap.get(DcMotorEx.class, "Slide_top");

        //Make the encoder value be postive








//        front_led_green = hardwareMap.get(LED.class, "front_led_green");
//        front_led_red = hardwareMap.get(LED.class, "front_led_red");
//        rear_led_green = hardwareMap.get(LED.class, "rear_led_green");
//        rear_led_red = hardwareMap.get(LED.class, "rear_led_red");
        Right_handle = hardwareMap.get(Servo.class, "Right_handle");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Left_handle = hardwareMap.get(Servo.class, "Left_handle");

//        arm_grab = hardwareMap.get(Servo.class, "arm_grab");
        //  Intake_rot = hardwareMap.get(Servo.class, "Intake_rot");
        //  Intake = hardwareMap.get(CRServo.class, "Intake");
        //  Intake_handle = hardwareMap.get(Servo.class, "Intake_handle");
        Gearbox = hardwareMap.get(Servo.class, "Gearbox");

        Claw_color=hardwareMap.get(ColorSensor.class, "Claw_color");

//        Intake_color = hardwareMap.get(ColorSensor.class, "Intake_color");
        Arm_encoder= hardwareMap.get(AnalogInput.class, "Arm_encoder");

        basket_dist = hardwareMap.get(DistanceSensor.class,"basket_dist");
        bar_dist = hardwareMap.get(DistanceSensor.class,"bar_dist");

//        Webcam1=hardwareMap.get(WebcamName.class, "Webcam 1");





        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide_bot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide_top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        Back_led.setPwmRange(500,250);

        Slide_bot.setDirection(DcMotorSimple.Direction.REVERSE);
        Slide_top.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuparameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(imuparameters);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();


        Arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        Slide_bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide_bot.setTargetPosition(0);
        Slide_bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide_bot.setVelocity(0);

        Slide_top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide_top.setTargetPosition(0);
        Slide_top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide_top.setVelocity(0);




    }
}
