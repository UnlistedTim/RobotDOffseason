

package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="OffseasonIntake", group="AA")

public class IntakeTestOff extends LinearOpMode {


    CRServo Lefttake;
    CRServo Righttake;
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

            if (gamepad1.right_stick_y < -0.2 || gamepad1.right_stick_y > 0.2){
                Lefttake.setPower(-gamepad1.right_stick_y);
                Righttake.setPower(gamepad1.right_stick_y);
            }
            else{
                Lefttake.setPower(0);
                Righttake.setPower(0);
            }


        }



    }




    void Hwinit() {
        Lefttake = hardwareMap.get(CRServo.class, "Lefttake");
        Righttake = hardwareMap.get(CRServo.class, "Righttake");



        Lefttake.setPower(0);
        Righttake.setPower(0);

    }


}