

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="OffTimATeleop", group="AA")

public class OffATeleop extends LinearOpMode {

    //left and right CR intake servos
    CRServo Lefttake;
    CRServo Righttake;

    // Control hub side servos
    Servo Takeblocker;

    Servo Inhandle;

    // Diffy servos exapnsion hub side

    Servo Lefthandle;

    Servo Righthandle;

    Servo Claw;



    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;

    DcMotorEx Leftarm;
    DcMotorEx Rightarm;

    DcMotorEx Slide;
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


            robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, 1);


        }



    }




    void Hwinit() {
        Lefttake = hardwareMap.get(CRServo.class, "Lefttake");
        Righttake = hardwareMap.get(CRServo.class, "Righttake");

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        Leftarm = hardwareMap.get(DcMotorEx.class, "Leftarm");
        Rightarm = hardwareMap.get(DcMotorEx.class, "Rightarm");
        Slide = hardwareMap.get(DcMotorEx.class, "Slide");

        Takeblocker = hardwareMap.get(Servo.class, "Takeblocker");
        Inhandle = hardwareMap.get(Servo.class,"Inhandle");

        Lefthandle = hardwareMap.get(Servo.class, "Lefthandle");
        Righthandle = hardwareMap.get(Servo.class, "Lefthandle");

        Claw = hardwareMap.get(Servo.class, "Claw");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        Lefttake.setPower(0);
        Righttake.setPower(0);

    }

    public void robot_centric(double iy, double ix, double irx, double ratio) {
        double y = -iy;
        double x = ix * 1.1; // Counteract imperfect strafing
        double rx = irx * 0.75; // 0.75
        double drivinginput= Math.abs(y) + Math.abs(x) + Math.abs(rx);
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


}