package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



@TeleOp

public class thing extends LinearOpMode {
    private ColorRangeSensor Distance;
    private Servo Servo;
    private DigitalChannel Touch;
    private DcMotor Motor;

    @Override

    public void runOpMode() {
        Distance = hardwareMap.get(ColorRangeSensor.class, "DISTANCE");
        Servo = hardwareMap.get(Servo.class, "SERVO");
        Touch = hardwareMap.get(DigitalChannel.class, "TOUCH");
        Motor = hardwareMap.get(DcMotor.class, "johnSmithMotor");

        boolean Toggle;
        boolean PreviousValue;

        Toggle = false;
        PreviousValue = false;

        Touch.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while (opModeIsActive()) {
            if (!PreviousValue && Touch.getState()) {
                Toggle = !Toggle;
            }
            PreviousValue = Touch.getState();
            if (Toggle) {
                Motor.setPower(0);
                Servo.setPosition(1);
            } else {
                Motor.setPower(1.1 - (Distance.getDistance(DistanceUnit.CM)/10));
                Servo.setPosition(Distance.getDistance(DistanceUnit.CM)/10);
            }
            telemetry.addData("Distance (cm)", Distance.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}