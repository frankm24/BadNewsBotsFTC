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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(group="Frank's Programs")
public class ControlTeleOp extends LinearOpMode {


    //Elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    //Declare motors
    private DcMotor back_left;
    private DcMotor front_left;
    private DcMotor back_right;
    private DcMotor front_right;

    private Servo linear;
    private Servo claw;

    private DcMotorEx flywheel;
    private Servo pusher;
    private ModernRoboticsI2cRangeSensor rangeSensor;


    //Encoder settings
    final int FlywheelTargetSpeed = -2400; //ticks per second

    //Button code
    boolean RevStatus = false;
    boolean ADebounce = false;
    double ADebounceTime = 0;
    boolean XDebounce = false;
    double ADebounceTimer = 0.5;
    double XDebounceTimer = 2;
    double XDebounceTime = 0;

    //Events code

    //Maths lol
    static final double sqrt2over2 = (Math.sqrt(2)) / 2;

    public void smartWait(double time) {
        double t = getRuntime();
        while (true) {
            if (getRuntime() - t >= time) {
                return;
            }
        }
    }

    public void enableGamepadControl() {
        Points points = new Points();
        Drive drive = new Drive();

        //Odometry.startTracking();
        drive.setMotors(back_left, front_left, back_right, front_right);

        String LeftStickInputDirection = "";
        telemetry.addData("Left Stick Input Direction", LeftStickInputDirection);
        String RightStickInputDirection = "";
        telemetry.addData("Right Stick Input Direction", RightStickInputDirection);
        telemetry.update();

        while (opModeIsActive()) {

            float LeftStickY = -1 * gamepad1.left_stick_y;  //To use, press start and A on gamepad
            float LeftStickX = gamepad1.left_stick_x;
            float RightStickY = -1 * gamepad1.right_stick_y;
            float RightStickX = gamepad1.right_stick_x;
            boolean A = gamepad1.a;
            boolean X = gamepad1.x;
            boolean Y = gamepad1.y;
            boolean linear_pos = false;
            boolean claw_pos = false;


            if (X && XDebounce == false) {
                /*
                telemetry.addData("X Detected", true);
                XDebounce = true;
                double targetPos = pusher.getPosition() + 0.23;
                pusher.setPosition(targetPos);
                telemetry.addLine("Position set");
                telemetry.update();
                smartWait(2);
                targetPos = pusher.getPosition() - 0.23;
                telemetry.addLine("rotating back");
                telemetry.update();
                pusher.setPosition(targetPos)
                 */
                if (linear_pos) {
                    linear.setPosition(1/3);
                    linear_pos = true;
                }
                else {
                    linear.setPosition(2/3);
                    linear_pos = false;
                }

            }
            if (Y) {
                if (claw_pos) {
                    claw.setPosition(2/3);
                    claw_pos = true;
                }
                else {
                    claw.setPosition(1 / 3);
                    claw_pos = false;
                }
            }
            if (A) {
                if (RevStatus == false && ADebounce == false) {
                    flywheel.setVelocity(FlywheelTargetSpeed);
                    RevStatus = true;
                    ADebounce = true;
                    ADebounceTime = getRuntime();
                }
                else if (RevStatus == true && ADebounce == false) {
                    flywheel.setVelocity(0);
                    RevStatus = false;
                    ADebounce = true;
                    ADebounceTime = getRuntime();
                }
            }
            else {
                telemetry.addData("X Detected", false);
            }
            if (getRuntime() - ADebounceTime >= ADebounceTimer) {
                ADebounce = false;
            }
            if (getRuntime() - XDebounceTime >= XDebounceTimer) {
                XDebounce = false;
            }

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

            //rangeSensor.initialize();

            //telemetry.addData("Ultrasonic sensor", rangeSensor.getDistance(DistanceUnit.METER));
            //telemetry.addData("Encoder position: ", back_left.getCurrentPosition());
            telemetry.addData("Left Stick Input Direction", LeftStickInputDirection);
            telemetry.addData("LeftStickX", LeftStickX);
            telemetry.addData("LeftStickY", LeftStickY);
            telemetry.addData("Right Stick Input Direction", RightStickInputDirection);
            telemetry.addData("RightStickX", RightStickX);
            telemetry.addData("RightStickY", RightStickY);
            //telemetry.addData("Flywheel velocity: ", flywheel.getVelocity());
            //telemetry.addData("Pusher position", pusher.getPosition());
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
        //flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        //pusher = hardwareMap.get(Servo.class, "pusher");
        //rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor");

        linear = hardwareMap.get(Servo.class, "linear");
        claw = hardwareMap.get(Servo.class, "claw");

        // Reverse the motors that runs backwards (LEFT SIDE)
        front_right.setDirection(DcMotor.Direction.REVERSE);
        front_left.setDirection(DcMotor.Direction.REVERSE);

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


