

package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Kevin Demo", group="Demo")

public class KevinCode extends LinearOpMode {


    DcMotorEx leftFront, rightFront, leftBack, rightBack,Leftarm, Rightarm, Slide;
    Servo Takeblocker, Inhandle, Lefthandle, Righthandle, Claw, Sweep;

    CRServo Lefttake, Righttake;
    DistanceSensor DisT;
    ColorSensor ColorT;

    DigitalChannel TouchT;

    IMU ImuT;

    AnalogInput Armencoder;
    double Motorpower=0.5, Servoposition=0.1;
    int runtopositon1=500;


    @Override
    public void runOpMode() {

        Hwinit();
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x * 1.1;
            double rx = gamepad1.left_stick_x;
            double denominator = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx),1);
            double leftFrontPower =(y + x +rx)/denominator;
            double leftBackPower=(y - x + rx)/denominator;
            double rightFrontPower=(y - x - rx)/denominator;
            double rightBackPower=(y + x - rx)/denominator;
            leftBack.setPower(leftBackPower);
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);





        }



    }






    void Hwinit() {
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        Leftarm = hardwareMap.get(DcMotorEx.class, "Leftarm");
        Rightarm = hardwareMap.get(DcMotorEx.class, "Rightarm");
        Slide = hardwareMap.get(DcMotorEx.class, "Slide");

        Lefttake = hardwareMap.get(CRServo.class, "Lefttake");
        Takeblocker = hardwareMap.get(Servo.class, "Takeblocker");
        Righttake = hardwareMap.get(CRServo.class, "Righttake");
        Inhandle = hardwareMap.get(Servo.class, "Inhandle");
        Righthandle = hardwareMap.get(Servo.class, "Righthandle");
        Lefthandle = hardwareMap.get(Servo.class, "Lefthandle");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Sweep = hardwareMap.get(Servo.class, "Sweep");
        Armencoder = hardwareMap.get(AnalogInput.class, "Armencoder");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }


}