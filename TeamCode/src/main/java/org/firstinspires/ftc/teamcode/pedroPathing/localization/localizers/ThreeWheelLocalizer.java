package org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Encoder;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Matrix;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.util.NanoTimer;

/**
 * This is the ThreeWheelLocalizer class. This class extends the Localizer superclass and is a
 * localizer that uses the three wheel odometry set up. The diagram below, which is modified from
 * Road Runner, shows a typical set up.
 *
 * The view is from the top of the robot looking downwards.
 *
 * left on robot is the y positive direction
 *
 * forward on robot is the x positive direction
 *
 *                         forward (x positive)
 *                                △
 *                                |
 *                                |
 *                         /--------------\
 *                         |              |
 *                         |              |
 *                         | ||        || |
 *  left (y positive) <--- | ||        || |  
 *                         |     ____     |
 *                         |     ----     |
 *                         \--------------/
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/2/2024
 */
@Config
public class ThreeWheelLocalizer extends Localizer {
    private HardwareMap hardwareMap;
    private Pose startPose;
    private Pose displacementPose;
    private Pose currentVelocity;
    private Matrix prevRotationMatrix;
    private NanoTimer timer;
    private long deltaTimeNano;
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder strafeEncoder;
    private Pose leftEncoderPose;
    private Pose rightEncoderPose;
    private Pose strafeEncoderPose;
    private double totalHeading;
    public static double FORWARD_TICKS_TO_INCHES = 0.00192;//8192 * 1.37795 * 2 * Math.PI * 0.5008239963;
    public static double STRAFE_TICKS_TO_INCHES = 0.00192;//8192 * 1.37795 * 2 * Math.PI * 0.5018874659;
    public static double TURN_TICKS_TO_RADIANS = 0.00192;//8192 * 1.37795 * 2 * Math.PI * 0.5;

    /**
     * This creates a new ThreeWheelLocalizer from a HardwareMap, with a starting Pose at (0,0)
     * facing 0 heading.
     *
     * @param map the HardwareMap
     */
    public ThreeWheelLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    /**
     * This creates a new ThreeWheelLocalizer from a HardwareMap and a Pose, with the Pose
     * specifying the starting pose of the localizer.
     *
     * @param map the HardwareMap
     * @param setStartPose the Pose to start from
     */
    public ThreeWheelLocalizer(HardwareMap map, Pose setStartPose) {
        // TODO: replace these with your encoder positions
//        leftEncoderPose = new Pose(-18.5/25.4 - 0.1, 164.4/25.4, 0);
//        rightEncoderPose = new Pose(-18.4/25.4 - 0.1, -159.6/25.4, 0);
//        strafeEncoderPose = new Pose(0*(-107.9/25.4+8)+-107.9/25.4+0.25, -1.1/25.4-0.23, Math.toRadians(90));

        leftEncoderPose = new Pose(160/25.4, 101.6/25.4, 0);
        rightEncoderPose = new Pose(160/25.4, -101.6/25.4, 0);
        strafeEncoderPose = new Pose(152.4/25.4, -1.1/25.4-0.23, Math.toRadians(90));
        hardwareMap = map;

        // TODO: replace these with your encoder ports  //rightBack, leftBack
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftBack"));
        strafeEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightBack"));

        // TODO: reverse any encoders necessary
        leftEncoder.setDirection(Encoder.REVERSE);
        rightEncoder.setDirection(Encoder.REVERSE);
        strafeEncoder.setDirection(Encoder.REVERSE);

        setStartPose(setStartPose);
        timer = new NanoTimer();
        deltaTimeNano = 1;
        displacementPose = new Pose();
        currentVelocity = new Pose();
        totalHeading = 0;

        resetEncoders();
    }

    /**
     * This returns the current pose estimate.
     *
     * @return returns the current pose estimate as a Pose
     */
    @Override
    public Pose getPose() {
        return MathFunctions.addPoses(startPose, displacementPose);
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Pose
     */
    @Override
    public Pose getVelocity() {
        return currentVelocity.copy();
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Vector
     */
    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getVector();
    }

    /**
     * This sets the start pose. Changing the start pose should move the robot as if all its
     * previous movements were displacing it from its new start pose.
     *
     * @param setStart the new start pose
     */
    @Override
    public void setStartPose(Pose setStart) {
        startPose = setStart;
    }

    /**
     * This sets the Matrix that contains the previous pose's heading rotation.
     *
     * @param heading the rotation of the Matrix
     */
    public void setPrevRotationMatrix(double heading) {
        prevRotationMatrix = new Matrix(3,3);
        prevRotationMatrix.set(0, 0, Math.cos(heading));
        prevRotationMatrix.set(0, 1, -Math.sin(heading));
        prevRotationMatrix.set(1, 0, Math.sin(heading));
        prevRotationMatrix.set(1, 1, Math.cos(heading));
        prevRotationMatrix.set(2, 2, 1.0);
    }

    /**
     * This sets the current pose estimate. Changing this should just change the robot's current
     * pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose) {
        displacementPose = MathFunctions.subtractPoses(setPose, startPose);
        resetEncoders();
    }

    /**
     * This updates the elapsed time timer that keeps track of time between updates, as well as the
     * change position of the Encoders. Then, the robot's global change in position is calculated
     * using the pose exponential method.
     */
    @Override
    public void update() {
        deltaTimeNano = timer.getElapsedTime();
        timer.resetTimer();

        updateEncoders();
        Matrix robotDeltas = getRobotDeltas();
        Matrix globalDeltas;
        setPrevRotationMatrix(getPose().getHeading());

        Matrix transformation = new Matrix(3,3);
        if (Math.abs(robotDeltas.get(2, 0)) < 0.001) {
            transformation.set(0, 0, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(0, 1, -robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 0, robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 1, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(2, 2, 1.0);
        } else {
            transformation.set(0, 0, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(0, 1, (Math.cos(robotDeltas.get(2, 0)) - 1.0) / robotDeltas.get(2, 0));
            transformation.set(1, 0, (1.0 - Math.cos(robotDeltas.get(2, 0))) / robotDeltas.get(2, 0));
            transformation.set(1, 1, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(2, 2, 1.0);
        }

        globalDeltas = Matrix.multiply(Matrix.multiply(prevRotationMatrix, transformation), robotDeltas);

        displacementPose.add(new Pose(globalDeltas.get(0, 0), globalDeltas.get(1, 0), globalDeltas.get(2, 0)));
        currentVelocity = new Pose(globalDeltas.get(0, 0) / (deltaTimeNano / Math.pow(10.0, 9)), globalDeltas.get(1, 0) / (deltaTimeNano / Math.pow(10.0, 9)), globalDeltas.get(2, 0) / (deltaTimeNano / Math.pow(10.0, 9)));

        totalHeading += globalDeltas.get(2, 0);
    }

    /**
     * This updates the Encoders.
     */
    public void updateEncoders() {
        leftEncoder.update();
        rightEncoder.update();
        strafeEncoder.update();
    }

    /**
     * This resets the Encoders.
     */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
        strafeEncoder.reset();
    }

    /**
     * This calculates the change in position from the perspective of the robot using information
     * from the Encoders.
     *
     * @return returns a Matrix containing the robot relative movement.
     */
    public Matrix getRobotDeltas() {
        Matrix returnMatrix = new Matrix(3,1);
        // x/forward movement
        returnMatrix.set(0,0, FORWARD_TICKS_TO_INCHES * ((rightEncoder.getDeltaPosition() * leftEncoderPose.getY() - leftEncoder.getDeltaPosition() * rightEncoderPose.getY()) / (leftEncoderPose.getY() - rightEncoderPose.getY())));
        //y/strafe movement
        returnMatrix.set(1,0, STRAFE_TICKS_TO_INCHES * (strafeEncoder.getDeltaPosition() - strafeEncoderPose.getX() * ((rightEncoder.getDeltaPosition() - leftEncoder.getDeltaPosition()) / (leftEncoderPose.getY() - rightEncoderPose.getY()))));
        // theta/turning
        returnMatrix.set(2,0, TURN_TICKS_TO_RADIANS * ((rightEncoder.getDeltaPosition() - leftEncoder.getDeltaPosition()) / (leftEncoderPose.getY() - rightEncoderPose.getY())));
        return returnMatrix;
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    public double getTotalHeading() {
        return totalHeading;
    }

    /**
     * This returns the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    public double getForwardMultiplier() {
        return FORWARD_TICKS_TO_INCHES;
    }

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    public double getLateralMultiplier() {
        return STRAFE_TICKS_TO_INCHES;
    }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    public double getTurningMultiplier() {
        return TURN_TICKS_TO_RADIANS;
    }

    /**
     * This does nothing since this localizer does not use the IMU.
     */
    public void resetIMU() {
    }
}
