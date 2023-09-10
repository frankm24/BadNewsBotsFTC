package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;

import badnewsbots.hardware.GamepadEx;

@TeleOp(group = "Testing")
public class EpicServoTest extends LinearOpMode {
    private Servo servo;
    private Servo servo1;
    private GamepadEx smartGamepad;
    private final List<Servo> servosToTest = new ArrayList<>();

    private final double SERVO_MAX_POS = 1.0;
    private final double SERVO_INC = 0.01;
    private double currentTargetPosition = 0;
    private Servo currentServo = servo;
    private ListIterator<Servo>  servoListIterator = servosToTest.listIterator();

    @Override
    public void runOpMode() {
        smartGamepad = new GamepadEx(gamepad1);
        servo = hardwareMap.get(Servo.class, "gripper");
        servo1 = hardwareMap.get(Servo.class, "wrist");
        servosToTest.add(servo);
        servosToTest.add(servo1);

        telemetry.addLine("Initialized.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            smartGamepad.update();

            if (smartGamepad.y_pressed) {
                if (servoListIterator.hasNext()) {
                    currentServo = servoListIterator.next();
                } else {
                    servoListIterator = servosToTest.listIterator();
                    currentServo = servoListIterator.next();
                }
            }
            if (smartGamepad.a_pressed) {
                currentTargetPosition = 0;
            }
            if (smartGamepad.dpad_up_pressed) {
                currentTargetPosition = Math.min(currentTargetPosition + SERVO_INC, SERVO_MAX_POS);
            }
            if (smartGamepad.dpad_down_pressed) {
                currentTargetPosition = Math.max(0, currentTargetPosition - SERVO_INC);
            }

            currentServo.setPosition(currentTargetPosition);

            //-3251
            telemetry.addData("Servo being adjusted: ", currentServo.getPortNumber());
            for (Servo servo : servosToTest) {
                telemetry.addData("Servo " + servo.getPortNumber() + " position: ", servo.getPosition());
            }
            telemetry.update();
        }
    }
}

