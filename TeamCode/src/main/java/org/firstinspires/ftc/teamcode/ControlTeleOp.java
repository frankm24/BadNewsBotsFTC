package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@TeleOp
public class ControlTeleOp extends LinearOpMode {

    //Elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    //Declare motors
    private DcMotor back_left;
    private DcMotor front_left;
    private DcMotor back_right;
    private DcMotor front_right;
    private DcMotor[] motors = {back_left, front_left, front_right, back_right};

    private Servo arm;
    private Servo door;

    private DcMotorEx intake;

    private BNO055IMU imu;
    private Servo pusher;


    //settings
    final int FlywheelTargetSpeed = 1500; //ticks per second
    float SpeedMultiplier = 0.5f; //scale movement speed

    //Button code
    double ADebounceTimer = 0.3; //seconds
    double XDebounceTimer = 0.3;
    double YDebounceTimer = 0.3;

    boolean RevStatus = false;
    boolean doorClosed = false;
    boolean servoUp = false;
    boolean isArmInMotion = false;
    boolean ADebounce = false;
    double ADebounceTime = 0;
    boolean XDebounce = false;
    double XDebounceTime = 0;
    boolean YDebounce = false;
    double YDebounceTime = 0;

    //Events code

    //Maths lol
    //static final double sqrt2over2 = (Math.sqrt(2)) / 2;

    public void smartWait(double time) {
        double t = getRuntime();
        while (true) {
            if (getRuntime() - t >= time) {
                return;
            }
        }
    }

    public void enableGamepadControl() {
        //Points points = new Points();
        //Drive drive = new Drive();
        //drive.setMotors(back_left, front_left, back_right, front_right);

        String LeftStickInputDirection = "";
        telemetry.addData("Left Stick Input Direction", LeftStickInputDirection);
        String RightStickInputDirection = "";
        telemetry.addData("Right Stick Input Direction", RightStickInputDirection);
        telemetry.update();

        while (opModeIsActive() && !isStopRequested()) {
            /*
            wip version for devin *sigh*:
            if (X && !isArmInMotion) {
                isArmInMotion = true;
                telemetry.addLine("Arm moving.");
                door.close();
                arm.setPosition(0.5);
                smartWait(<Time it takes for servo to rise>)
                door.open();
                smartWait(3000) or waitForButtonPress();
                arm.setPosition(-0.06);
                smartWait(<Time it takes for servo to rise>);
                isArmInMotion = false;
            }

             */

            float LeftStickY = -1 * gamepad1.left_stick_y * SpeedMultiplier;  //To use, press start and A on gamepad
            float LeftStickX = gamepad1.left_stick_x * SpeedMultiplier;
            float RightStickY = -1 * gamepad1.right_stick_y * SpeedMultiplier;
            float RightStickX = gamepad1.right_stick_x * SpeedMultiplier;
            boolean A = gamepad1.a;
            boolean X = gamepad1.x;
            boolean Y = gamepad1.y;

            if (Y && !YDebounce) {
                YDebounce = true;
                SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

                Pose2d startPose = new Pose2d(0, 0, 0);

                drive.setPoseEstimate(startPose);

                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                        .back(24)
                        .build();

                drive.followTrajectorySequence(trajSeq);


                arm.setPosition(0.5);
                sleep(3000);
                arm.setPosition(-0.35);
                YDebounce = false;
            }
            if (X && !XDebounce) {
                XDebounce = true;
                door.setPosition(0);
                sleep(100);
                arm.setPosition(0.78);
                sleep(3000);
                door.setPosition(0.65);
                sleep(1000);
                arm.setPosition(0);
                XDebounce = false;
            }
            if (A && !ADebounce) {
                telemetry.addLine("A detected");
                ADebounceTime = getRuntime();
                ADebounce = true;
                if (!RevStatus) {
                    intake.setVelocity(FlywheelTargetSpeed);
                    RevStatus = true;
                } else if (RevStatus) {
                    intake.setVelocity(0);
                    RevStatus = false;
                }
            }
            /*
            if (X && !XDebounce) {
                XDebounceTime = getRuntime();
                XDebounce = true;
                if (servoUp) {
                    telemetry.addLine("X detected");
                    arm.setPosition(-0.06);
                    servoUp = false;
                } else {
                    arm.setPosition(0.5);
                    servoUp = true;
                }
            }
            if (Y && !YDebounce) {
                telemetry.addLine("Y detected");
                YDebounceTime = getRuntime();
                YDebounce = true;
                if (!doorClosed) {
                    door.setPosition(0.65);
                    doorClosed = true;
                } else if (doorClosed) {
                    door.setPosition(1);
                    doorClosed = false;
                }
            }
            if (getRuntime() - XDebounceTime >= XDebounceTimer) {
                XDebounce = false;
            }
            if (getRuntime() - YDebounceTime >= YDebounceTimer) {
                YDebounce = false;
            }
             */
            if (getRuntime() - ADebounceTime >= ADebounceTimer) {
                ADebounce = false;
            }
            double denominator = Math.max(Math.abs(LeftStickY) + Math.abs(LeftStickX) + Math.abs(RightStickX), 1);
            double front_leftPower = (LeftStickY + LeftStickX + RightStickX) / denominator;
            double back_leftPower = (LeftStickY - LeftStickX + RightStickX) / denominator;
            double front_rightPower = (LeftStickY - LeftStickX - RightStickX) / denominator;
            double back_rightPower = (LeftStickY + LeftStickX - RightStickX) / denominator;
            front_left.setPower(front_leftPower);
            back_left.setPower(back_leftPower);
            front_right.setPower(front_rightPower);
            back_right.setPower(back_rightPower);
            //Old code in there... you don't want to look...
            /*
            if (LeftStickX == 0 && LeftStickY == 0) {
                drive.setMotorsToZero();
                LeftStickInputDirection = "none";
            }
            else if (LeftStickX >= 0 && LeftStickY > 0) { //Quadrant I
                if (points.distanceFormula(LeftStickX, LeftStickY, 0, 1) < //Closer to fd than midp
                        points.distanceFormula(LeftStickX, LeftStickY, sqrt2over2, sqrt2over2)) {
                    drive.forward(drive.getPowerFromMagnitude(LeftStickX, LeftStickY));
                    LeftStickInputDirection = "forward";
                }
                else if (points.distanceFormula(LeftStickX, LeftStickY, 1, 0) < //Closer to right than midp
                        points.distanceFormula(LeftStickX, LeftStickY, sqrt2over2, sqrt2over2)) {
                    drive.right(drive.getPowerFromMagnitude(LeftStickX, LeftStickY));
                    LeftStickInputDirection = "right";
                }
                else {
                    drive.forwardRight(drive.getPowerFromMagnitude(LeftStickX, LeftStickY));
                    LeftStickInputDirection = "forward_right";
                }
            }
            else if (LeftStickX < 0 && LeftStickY >= 0) { //Quadrant II
                if (points.distanceFormula(LeftStickX, LeftStickY, 0, 1) < //Closer to fd than midp
                        points.distanceFormula(LeftStickX, LeftStickY, -1 * sqrt2over2, sqrt2over2)) {
                    drive.forward(drive.getPowerFromMagnitude(LeftStickX, LeftStickY));
                    LeftStickInputDirection = "forward";
                }
                else if (points.distanceFormula(LeftStickX, LeftStickY, -1, 0) < //Closer to left than midp
                        points.distanceFormula(LeftStickX, LeftStickY, -1 * sqrt2over2, sqrt2over2)) {
                    drive.left(drive.getPowerFromMagnitude(LeftStickX, LeftStickY));
                    LeftStickInputDirection = "left";
                }
                else {
                    drive.forwardLeft(drive.getPowerFromMagnitude(LeftStickX, LeftStickY));
                    LeftStickInputDirection = "forward_left";
                }
            }
            else if (LeftStickX <= 0 && LeftStickY < 0) { //Quadrant III
                if (points.distanceFormula(LeftStickX, LeftStickY, 0, -1) < //Closer to bkwd than midp
                        points.distanceFormula(LeftStickX, LeftStickY, -1 * sqrt2over2, -1 * sqrt2over2)) {
                    drive.backward(drive.getPowerFromMagnitude(LeftStickX, LeftStickY));
                    LeftStickInputDirection = "backward";
                }
                else if (points.distanceFormula(LeftStickX, LeftStickY, -1, 0) < //Closer to left than midp
                        points.distanceFormula(LeftStickX, LeftStickY, -1 * sqrt2over2, -1 * sqrt2over2)) {
                    drive.left(drive.getPowerFromMagnitude(LeftStickX, LeftStickY));
                    LeftStickInputDirection = "left";
                }
                else {
                    drive.backLeft(drive.getPowerFromMagnitude(LeftStickX, LeftStickY));
                    LeftStickInputDirection = "back_left";
                }
            }
            else { //Quadrant IV
                if (points.distanceFormula(LeftStickX, LeftStickY, 0, -1) < //Closer to bkwd than midp
                        points.distanceFormula(LeftStickX, LeftStickY, sqrt2over2, -1 * sqrt2over2)) {
                    drive.backward(drive.getPowerFromMagnitude(LeftStickX, LeftStickY));
                    LeftStickInputDirection = "backward";
                }
                else if (points.distanceFormula(LeftStickX, LeftStickY, 1, 0) < //Closer to right than midp
                        points.distanceFormula(LeftStickX, LeftStickY, sqrt2over2, -1 * sqrt2over2)) {
                    drive.right(drive.getPowerFromMagnitude(LeftStickX, LeftStickY));
                    LeftStickInputDirection = "right";
                }
                else {
                    drive.backRight(drive.getPowerFromMagnitude(LeftStickX, LeftStickY));
                    LeftStickInputDirection = "back_right";
                }
            }
            //Right Stick
            if (RightStickX == 0 && RightStickY == 0) {
                drive.setMotorsToZero();
                RightStickInputDirection = "none";
            }
            else if (RightStickX >= 0 && RightStickY > 0) { //Quadrant I
                if (points.distanceFormula(RightStickX, RightStickY, 0, 1) < //Closer to fd than midp
                        points.distanceFormula(RightStickX, RightStickY, sqrt2over2, sqrt2over2)) {
                    RightStickInputDirection = "forward";
                }
                else if (points.distanceFormula(RightStickX, RightStickY, 1, 0) < //Closer to right than midp
                        points.distanceFormula(RightStickX, RightStickY, sqrt2over2, sqrt2over2)) {
                    drive.turnRight(drive.getPowerFromMagnitude(RightStickX, RightStickY));
                    RightStickInputDirection = "right";
                }
                else {
                    RightStickInputDirection = "forward_right";
                }

            }
            else if (RightStickX < 0 && RightStickY >= 0) { //Quadrant II
                if (points.distanceFormula(RightStickX, RightStickY, 0, 1) < //Closer to fd than midp
                        points.distanceFormula(RightStickX, RightStickY, -1 * sqrt2over2, sqrt2over2)) {
                    RightStickInputDirection = "forward";
                }
                else if (points.distanceFormula(RightStickX, RightStickY, -1, 0) < //Closer to left than midp
                        points.distanceFormula(RightStickX, RightStickY, -1 * sqrt2over2, sqrt2over2)) {
                    drive.turnLeft(drive.getPowerFromMagnitude(RightStickX, RightStickY));
                    RightStickInputDirection = "left";
                }
                else {
                    RightStickInputDirection = "forward_left";
                }
            }
            else if (RightStickX <= 0 && RightStickY < 0) { //Quadrant III
                if (points.distanceFormula(RightStickX, RightStickY, 0, -1) < //Closer to bkwd than midp
                        points.distanceFormula(RightStickX, RightStickY, -1 * sqrt2over2, -1 * sqrt2over2)) {
                    RightStickInputDirection = "backward";
                }
                else if (points.distanceFormula(RightStickX, RightStickY, -1, 0) < //Closer to left than midp
                        points.distanceFormula(RightStickX, RightStickY, -1 * sqrt2over2, -1 * sqrt2over2)) {
                    drive.turnLeft(drive.getPowerFromMagnitude(RightStickX, RightStickY));
                    RightStickInputDirection = "left";
                }
                else {
                    RightStickInputDirection = "back_left";
                }
            }
            else { //Quadrant IV
                if (points.distanceFormula(RightStickX, RightStickY, 0, -1) < //Closer to bkwd than midp
                        points.distanceFormula(RightStickX, RightStickY, sqrt2over2, -1 * sqrt2over2)) {
                    RightStickInputDirection = "backward";
                }
                else if (points.distanceFormula(RightStickX, RightStickY, 1, 0) < //Closer to right than midp
                        points.distanceFormula(RightStickX, RightStickY, sqrt2over2, -1 * sqrt2over2)) {
                    drive.turnRight(drive.getPowerFromMagnitude(RightStickX, RightStickY));
                    RightStickInputDirection = "right";
                }
                else {
                    RightStickInputDirection = "back_right";
                }
            }
            */
            telemetry.addData("IMU Data", imu.getAngularOrientation());
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() {
        // hardwareMap.get stuff
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        try {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
        } catch (IllegalArgumentException e) {
            telemetry.addLine("expansion hub not working");
            telemetry.update();
        }
        arm = hardwareMap.get(Servo.class, "arm");
        door = hardwareMap.get(Servo.class, "door");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Reverse the motors that runs backwards (LEFT SIDE)
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);

        //flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Odometry.setEncoders(front_left, front_right, back_right);
        // Wait for play button to be pressed
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        //Where stuff happens
        enableGamepadControl();
       // Odometry.stopTracking();
    }
}


