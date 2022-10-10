package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;


@TeleOp
public class ServoTest extends LinearOpMode {

    private Servo servo;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo0");
        servo.setDirection(Servo.Direction.FORWARD);
        ServoController controller = servo.getController();
        waitForStart();
        servo.setPosition(0);
        sleep(6000);
        servo.setPosition(1);
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Servo pos", servo.getPosition());
            telemetry.update();
            sleep(250);
        }

    }
}
