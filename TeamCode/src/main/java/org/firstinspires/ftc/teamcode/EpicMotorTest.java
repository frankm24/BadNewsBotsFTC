package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.List;

import badnewsbots.hardware.GamepadEx;

@TeleOp(group = "Testing")
public class EpicMotorTest extends LinearOpMode {
    private DcMotorEx motor;
    private DcMotorEx motor1;
    private GamepadEx smartGamepad;
    private double motorPower;
    private int motorTargetPosition;
    private final List<DcMotorEx> motorsToTest = new ArrayList<>();

    enum TestMode {
        POSITION,
        POWER
    }
    private TestMode currentTestMode;

    private final int minTicks = 100;
    private final int maxTicks = 3251;

    @Override
    public void runOpMode() {
        smartGamepad = new GamepadEx(gamepad1);
        motor = hardwareMap.get(DcMotorEx.class, "linearSlide1");
        motor1 = hardwareMap.get(DcMotorEx.class, "linearSlide2");
        //motorsToTest.add(motor);
        motorsToTest.add(motor1);

        motorPower = 0;
        motorTargetPosition = 0;
        currentTestMode = TestMode.POWER;

        telemetry.addLine("Initialized.");
        for (int i = 0; i < motorsToTest.size(); i++) {
            telemetry.addData("Motor " + i + " PIDF Coeffs: ", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        }
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            smartGamepad.update();

            if (smartGamepad.y_pressed) {
                if (currentTestMode == TestMode.POWER) {
                    currentTestMode = TestMode.POSITION;
                    for (DcMotorEx motor : motorsToTest) {
                        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motor.setPower(0.5);
                        motorTargetPosition = minTicks;
                    }
                } else {
                    currentTestMode = TestMode.POWER;
                    for (DcMotorEx motor : motorsToTest) {
                        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorPower = 0;
                    }
                }
            }
            if (currentTestMode == TestMode.POWER) {
                if (smartGamepad.dpad_up) {
                    motorPower += 0.01;
                }
                if (smartGamepad.dpad_down) {
                    motorPower -= 0.01;
                }
                if (smartGamepad.a_pressed) {
                    motorPower = 0;
                }
                if (smartGamepad.b_pressed) {
                    motorPower = -0.1;
                }
                for (DcMotorEx motor : motorsToTest) {
                    motor.setPower(motorPower);
                }
            } else {
                if (smartGamepad.dpad_up) {
                    motorTargetPosition = Math.min(motorTargetPosition + 2, maxTicks);
                }
                if (smartGamepad.dpad_down) {
                    motorTargetPosition = Math.max(motorTargetPosition - 2, minTicks);
                }
                if (smartGamepad.a_pressed) {
                    motorTargetPosition = 0;
                }
                for (DcMotorEx motor : motorsToTest) {
                    motor.setTargetPosition(motorTargetPosition);
                }
            }
            //-3251
            telemetry.addData("Test mode: ", currentTestMode);
            for (int i = 0; i < motorsToTest.size(); i++) {
                telemetry.addData("Motor " + i + " position: ", motor.getCurrentPosition());
            }
            telemetry.addData("Motor power: ", motorPower);
            telemetry.addData("Motor target position: ", motorTargetPosition);
            telemetry.update();
        }
    }
}
