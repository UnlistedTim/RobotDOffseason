import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
@TeleOp
public class TankTele extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        // Declare our motors

        double drive;
        double turn;

        double right_power;
        double left_power;

        // Make sure your ID's match your configuration
        DcMotor front_left = hardwareMap.dcMotor.get("front_left");
        DcMotor front_right = hardwareMap.dcMotor.get("front_right");
        DcMotor rear_left = hardwareMap.dcMotor.get("rear_left");
        DcMotor rear_right = hardwareMap.dcMotor.get("rear_right");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
//        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        rear_right.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            drive = -gamepad1.right_stick_y;
            turn = -gamepad1.left_stick_x;

            left_power = Range.clip(drive - turn,-1,1);
            right_power = Range.clip(drive + turn,-1,1);

            front_left.setPower(left_power);
            front_right.setPower(right_power);
            rear_left.setPower(left_power);
            rear_right.setPower(right_power);

            telemetry.addData("leftpower",left_power);
            telemetry.addData("right power", right_power);
            telemetry.update();

        }
    }
}