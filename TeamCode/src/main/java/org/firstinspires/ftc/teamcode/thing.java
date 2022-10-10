package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class thing extends LinearOpMode {
    private DcMotor motor;
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class,"back_right");
        waitForStart();
        motor.setPower(1);
        sleep(10000);
    }
}
