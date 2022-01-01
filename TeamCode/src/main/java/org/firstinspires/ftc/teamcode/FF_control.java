package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "old_old_control", group = "tony")
public class FF_control extends LinearOpMode {

    private DcMotor leftfront;
    private DcMotor leftback;
    private DcMotor rightfront;
    private DcMotor rightback;

    private Servo linear;
    private Servo claw;
    /*
    public void drive(float typower, float txpower, float direc) {
        if (typower != 0) {
            if (txpower == 0) {
                leftfront.setPower(typower);
                leftback.setPower(typower);
                rightfront.setPower(-typower);
                rightback.setPower(-typower);
            } else {
                if (txpower * typower > 0) {
                    leftback.setPower(typower + txpower);
                    rightfront.setPower(-(typower + txpower));
                } else {
                    leftfront.setPower(typower - txpower);
                    rightback.setPower(-(typower - txpower));
                }
            }
        } else {
            leftfront.setPower(-txpower);
            leftback.setPower(txpower);
            rightfront.setPower(-txpower);
            rightback.setPower(txpower);
        }
        if (direc != 0) {
            leftfront.setPower(-direc);
            leftback.setPower(-direc);
            rightfront.setPower(-direc);
            rightback.setPower(-direc);
        }
    }
    */
    public void servo(boolean X, Boolean Y, Boolean linear_pos, Boolean claw_pos){
        if (X) {
            if (linear_pos) {
                linear.setPosition(linear.getPosition()+1/10);
                linear_pos = true;
            }
            else {
                linear.setPosition(linear.getPosition()-1/10);
                linear_pos = false;
            }
        }
        if (Y) {
            if (claw_pos) {
                claw.setPosition(claw.getPosition()+ 1/10);
                claw_pos = true;
            }
            else {
                claw.setPosition(claw.getPosition()- 1/10);
                claw_pos = false;
            }
        }
    }
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        float typower;
        float txpower;
        float direc;
        boolean linear_pos = false;
        boolean claw_pos = false;
        boolean X;
        boolean Y;

        //leftfront = hardwareMap.dcMotor.get("left front");
        //leftback = hardwareMap.dcMotor.get("left back");
        //rightfront = hardwareMap.dcMotor.get("right front");
        //rightback = hardwareMap.dcMotor.get("right back");

        linear = hardwareMap.get(Servo.class, "linear");
        claw = hardwareMap.get(Servo.class, "claw");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                typower = gamepad1.left_stick_y;
                txpower = gamepad1.left_stick_x;
                direc = gamepad1.right_stick_x;
                X = gamepad1.x;
                Y = gamepad1.y;

                //drive(typower, txpower, direc);
                servo(X, Y, linear_pos, claw_pos);
            }
        }
    }
}
