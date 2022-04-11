package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DeltaTick extends LinearOpMode {
    private DcMotor motor;
    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "linear");
        waitForStart();
        int start_ticks = motor.getCurrentPosition();
        while (opModeIsActive()) {
            telemetry.addData("Delta ticks: ", motor.getCurrentPosition());
        }
    }
}
