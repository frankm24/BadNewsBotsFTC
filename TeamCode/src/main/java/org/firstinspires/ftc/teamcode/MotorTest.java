package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "Testing")
public class MotorTest extends LinearOpMode {
    //Declare motors
    private DcMotor back_left;
    private DcMotor front_left;
    private DcMotor back_right;
    private DcMotor front_right;

    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        back_left = hardwareMap.get(DcMotor.class, "back_left");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");


        // Wait for play button to be pressed
        waitForStart();
        telemetry.addData("Status", "Running test");
        telemetry.update();

        front_left.setPower(1);
        Thread.sleep(1000);
        front_left.setPower(0);

        front_right.setPower(1);
        Thread.sleep(1000);
        front_right.setPower(0);

        back_left.setPower(1);
        Thread.sleep(1000);
        back_left.setPower(0);

        back_right.setPower(1);
        Thread.sleep(1000);
        back_right.setPower(0);
    }
}