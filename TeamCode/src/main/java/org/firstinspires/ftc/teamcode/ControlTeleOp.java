package org.firstinspires.ftc.teamcode;

import com.badnewsbots.ultimategoal.Points;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.badnewsbots.ultimategoal.Drive;

@TeleOp(group="Frank's Programs")
public class ControlTeleOp extends LinearOpMode {

    //Elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    //Declare motors
    private DcMotor back_left;
    private DcMotor front_left;
    private DcMotor back_right;
    private DcMotor front_right;

    private Servo arm;

    private DcMotorEx intake;
    private Servo pusher;
    private ModernRoboticsI2cRangeSensor rangeSensor;


    //Encoder settings
    final int FlywheelTargetSpeed = 1500; //ticks per second

    //Button code
    double ADebounceTimer = 0.3; //seconds
    double XDebounceTimer = 0.3;

    boolean RevStatus = false;
    boolean servoUp = false;
    boolean ADebounce = false;
    double ADebounceTime = 0;
    boolean XDebounce = false;
    double XDebounceTime = 0;

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

        servoUp = false;
        while (opModeIsActive() && !isStopRequested()) {

            float LeftStickY = -1 * gamepad1.left_stick_y;  //To use, press start and A on gamepad
            float LeftStickX = gamepad1.left_stick_x;
            float RightStickY = -1 * gamepad1.right_stick_y;
            float RightStickX = gamepad1.right_stick_x;
            boolean A = gamepad1.a;
            boolean X = gamepad1.x;

            if (X && !XDebounce) {
                XDebounceTime = getRuntime();
                XDebounce = true;
                if (servoUp) {
                    telemetry.addLine("X detected");
                    telemetry.update();
                    arm.setPosition(0);
                    servoUp = false;
                } else {
                    arm.setPosition(0.4);
                    servoUp = true;
                }
            }
            if (A && !ADebounce) {
                telemetry.addLine("A detected");
                telemetry.update();
                ADebounceTime = getRuntime();
                ADebounce = true;
                if (RevStatus == false) {
                    intake.setVelocity(FlywheelTargetSpeed);
                    RevStatus = true;
                }
                else if (RevStatus == true ) {
                    intake.setVelocity(0);
                    RevStatus = false;
                }
            }

            if (getRuntime() - ADebounceTime >= ADebounceTimer) {
                ADebounce = false;
            }
            if (getRuntime() - XDebounceTime >= XDebounceTimer) {
                XDebounce = false;
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

            telemetry.addData("Left Stick Input Direction", LeftStickInputDirection);
            telemetry.addData("LeftStickX", LeftStickX);
            telemetry.addData("LeftStickY", LeftStickY);
            telemetry.addData("Right Stick Input Direction", RightStickInputDirection);
            telemetry.addData("RightStickX", RightStickX);
            telemetry.addData("RightStickY", RightStickY);
            telemetry.update();

        }
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // hardwareMap.get stuff
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        arm = hardwareMap.get(Servo.class, "arm");

        // Reverse the motors that runs backwards (LEFT SIDE)
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);

        //flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Odometry.setEncoders(front_left, front_right, back_right);
        // Wait for play button to be pressed
        waitForStart();
        runtime.reset();
        //Where stuff happens
        enableGamepadControl();
       // Odometry.stopTracking();
    }
}


