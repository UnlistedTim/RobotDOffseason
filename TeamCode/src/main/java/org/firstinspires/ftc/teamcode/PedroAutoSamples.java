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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

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
public class PedroAutoSamples extends OpMode {

    public double[][] pidftable = new double[20][3];

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    Pose2d pp = new Pose2d(0, 0, 0);

    int ppp=0,ii=1,dd=2;

    BaseClass rbg;

    double pid ,power, ff;

    int rotatePos,slidePos,absrotategap;

    public  DcMotorEx Arm_right, Arm_left, Slide_top,Slide_bot;
    public  Servo Intake_rot,Intake_handle, Gearbox;
    public DigitalChannel Arm_touch;
    public CRServo Intake;
    public VoltageSensor voltageSensor;
    public ColorSensor Intake_color;

    public DistanceSensor bar_dist;
    public DistanceSensor basket_dist;



    int pidf_intake_up=0,pidf_intake_down=1, pidf_outtake_down=2,pidf_outtake_up=3, pidf_intake_idle = 4,
            pidf_hang_up = 5, pidf_hang2 = 6, pidf_hang3 = 7, pidf_outtake_spec = 8,
            pidf_outtake_spec_down = 9, pidf_outtake_spec1 = 10 , pidf_outtake_up2 = 11,
            pidf_intake_spec = 12, pidf_intake_spec2 = 13,pidf_intake_aspec=14,pidf_aspec_outtake=15,pidf_outtake_aspec_down=16,pidf_aintake_down=17, pidf_hang4 = 18,pidf_aouttake_up2;

    int rotateTarget=0;





    public LED front_led_red;
    public LED front_led_green;

    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightFront;
    public DcMotorEx rightBack;

    PIDController controller;

    final double ticks_in_degree = 8192/360;

    public LED rear_led_red;
    public LED rear_led_green;

    double rotateStartangle=0 ;

    double p = 0.0000, i = 0, d = 0.000 ,f = 0.12,k = 0.000035; //0.000035



    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8, 110, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(19, 123, Math.toRadians(315)); // 9, 132

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
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

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
        scorePreload = new Path(new BezierCurve(new Point(startPose), new Point(scoreControlPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

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

    public void pedrosample_preouttake() {


        pidfsetting(2325+50, pidf_outtake_up);



    }

    public void pedrosample_outtake() {
        move(0);
        Intake_handle.setPosition(0);
        Intake_rot.setPosition(0.55);
        pidfsetting(2325+50, pidf_outtake_up2);
        linearslide(2470-20, 2000);
        delay(2000);

        Intake.setPower(-0.26);//-0.28
        delay(300);
        Intake.setPower(0);
        // delay(100);
        Intake_handle.setPosition(0.3);// lift the handle for a temp  higher locaiton.
        delay(50);
        move(0.25);
        delay(100);
        linearslide(400, 2500);
        delay(200);
        move(0);
        Intake_handle.setPosition(0.37);
        delay(200);
       // linearslide(-10, 1500);
        //delay(200);
        pidfsetting(325-250 ,pidf_outtake_down);


    }


    public void move(double power) {
       leftFront .setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }
    

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                pedrosample_preouttake();
                follower.followPath(scorePreload,true);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    /* Score Preload */


                    pedrosample_outtake();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (Slide_top.getCurrentPosition() < 600 && -Arm_right.getCurrentPosition() < 600){
                        follower.followPath(grabPickup1,true);
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
//            case 8:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(follower.getPose().getX() > (parkPose.getX() - 1) && follower.getPose().getY() > (parkPose.getY() - 1)) {
//                    /* Put the claw in position to get a level 1 ascent */
//                    claw.startClaw();
//                    claw.closeClaw();
//
//                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
//                    setPathState(-1);
//                }
//            break;
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

    public void pidfsetting(int target, int index)

    {
        p=pidftable[index][ppp];
        i=pidftable[index][ii];
        d=pidftable[index][dd];
        rotateTarget=target;
        armrotatePIDF();
    }

    public void armrotatePIDF()

    {
        rotatePos = -Arm_right.getCurrentPosition();
        slidePos = Slide_top.getCurrentPosition();
        controller.setPID(p,i,d);
        pid = controller.calculate(rotatePos,rotateTarget);
        ff = Math.cos(Math.toRadians(rotatePos/ticks_in_degree +rotateStartangle)) * (f + k*slidePos) ;// target
        power = pid + ff;

//        if (power > 1.0) power = 0.98;
//        if (power < -1.0) power = -0.98;
        Arm_left.setPower(-power);
        Arm_right.setPower(power);// to be changed director.

    }

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

        follower = new Follower(hardwareMap);


        pidftable[pidf_intake_up][ppp]=0.0024;  pidftable[pidf_intake_up][ii]=0;  pidftable[pidf_intake_up][dd]=0.0001;
        pidftable[pidf_intake_idle][ppp]=0.003;  pidftable[pidf_intake_idle][ii]=0;  pidftable[pidf_intake_idle][dd]=0.00008;
        pidftable[pidf_intake_down][ppp]=0.002;  pidftable[pidf_intake_down][ii]=0;  pidftable[pidf_intake_down][dd]=0.00005;
        pidftable[pidf_outtake_up][ppp]=0.00037;  pidftable[pidf_outtake_up][ii]=0;  pidftable[pidf_outtake_up][dd]=0.000023;//
        pidftable[pidf_outtake_up2][ppp]=0.00075;  pidftable[pidf_outtake_up2][ii]=0.00012;  pidftable[pidf_outtake_up2][dd]=0.0001;//0.0025

        pidftable[pidf_outtake_down][ppp]=0.00035;  pidftable[pidf_outtake_down][ii]=0;  pidftable[pidf_outtake_down][dd]=0.00012;




        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");


//        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
//        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
//                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;



        Arm_right = hardwareMap.get(DcMotorEx.class, "Arm_right");
        Arm_left = hardwareMap.get(DcMotorEx.class, "Arm_left");
        Slide_bot = hardwareMap.get(DcMotorEx.class, "Slide_bot");
        Slide_top = hardwareMap.get(DcMotorEx.class, "Slide_top");

        front_led_green = hardwareMap.get(LED.class, "front_led_green");
        front_led_red = hardwareMap.get(LED.class, "front_led_red");
        rear_led_green = hardwareMap.get(LED.class, "rear_led_green");
        rear_led_red = hardwareMap.get(LED.class, "rear_led_red");

//        arm_grab = hardwareMap.get(Servo.class, "arm_grab");
        Intake_rot = hardwareMap.get(Servo.class, "Intake_rot");
        Intake = hardwareMap.get(CRServo.class, "Intake");
        Intake_handle = hardwareMap.get(Servo.class, "Intake_handle");
        Gearbox = hardwareMap.get(Servo.class, "Gearbox");

//        Intake_color = hardwareMap.get(ColorSensor.class, "Intake_color");
        Arm_touch = hardwareMap.get(DigitalChannel.class,"Arm_touch");

        basket_dist = hardwareMap.get(DistanceSensor.class,"basket_dist");
        bar_dist = hardwareMap.get(DistanceSensor.class,"bar_dist");

//        Webcam1=hardwareMap.get(WebcamName.class, "Webcam 1");





        Slide_bot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide_top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        controller = new PIDController(p, i, d);
        Intake_rot.setPosition(0.55);
        Gearbox.setPosition(0);




       // s0.0016



        follower.setStartingPose(startPose);

        buildPaths();

        telemetry.update();
        pause(500);
        Arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pause(500);

        telemetry.addLine("Ready for right bumper");
        telemetry.update();
//        while(true)
//        {
//            telemetry.addLine("In this loop");
//            telemetry.update();
//            pause(20);
//            if(gamepad1.right_bumper) break;
//        }
//        Slide_bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Slide_bot.setTargetPosition(0);
//        Slide_bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Slide_bot.setVelocity(0);
//
//        Slide_top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Slide_top.setTargetPosition(0);
//        Slide_top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Slide_top.setVelocity(0);
//        pause(500);
//        pidfsetting(-Arm_right.getCurrentPosition(),pidf_intake_idle);
//        delay(500);
//        telemetry.addLine("Done");
//        telemetry.update();

        // Set the claw to positions for init
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {

        telemetry.addData("Arm Pos", -Arm_right.getCurrentPosition());

        telemetry.addData("rotate targ",rotateTarget);

        telemetry.addData("arm power",power);

        telemetry.update();


        if (gamepad1.right_bumper){

            Intake_handle.setPosition(0.3);
            Intake_rot.setPosition(0.55);
            Slide_bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slide_bot.setTargetPosition(0);
            Slide_bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide_bot.setVelocity(0);

            Slide_top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slide_top.setTargetPosition(0);
            Slide_top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide_top.setVelocity(0);
            pause(300);
            pidfsetting((-Arm_right.getCurrentPosition()),pidf_intake_idle);
            delay(500);
            telemetry.addLine("Done");
            telemetry.update();


        }

    armrotatePIDF();

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
}
