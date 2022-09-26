package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class thing extends LinearOpMode {
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor flywheel;
    public void runOpMode() {
        backRight = hardwareMap.get(DcMotor.class,"back_right");
        backLeft = hardwareMap.get(DcMotor.class,"back_left");
        frontRight = hardwareMap.get(DcMotor.class,"front_right");
        frontLeft = hardwareMap.get(DcMotor.class,"front_left");
        flywheel = hardwareMap.get(DcMotor.class,"flywheel");
        waitForStart();
        powerFlywheel(1,10000);
    }
    public void powerFlywheel(double power,long duration) {
        flywheel.setPower(power);
        sleep(duration);
    }
}
