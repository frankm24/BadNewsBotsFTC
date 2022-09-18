package badnewsbots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

@TeleOp
public class LinearMechanismTest extends LinearOpMode {

    DcMotorEx motor;
    // 465mm // 1525 ticks
    // measured maximum (full extension): -1575
    // Trial 1: .305mm/tick
    // Trial 2: .303mm/tick
    // Trial 3: .302 mm/tick
    // Trial 4: .302mm/tick
    // Trial 5: .303mm/tick
    // avg: .303mm/tick


    @Override
    public void runOpMode() {
        int target_pos = -1525;
        motor = hardwareMap.get(DcMotorEx.class, "front_left");
        motor.setTargetPosition(target_pos);
        waitForStart();
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(-0.5); // Power will adjust down once target position is reached

        while (opModeIsActive()) {
            int motor_pos = motor.getCurrentPosition();
            if (motor_pos == target_pos) {
                telemetry.addLine("Target position reached. Motor will resist applied force until program stopped.");
            } else {
                telemetry.addLine("Running to target position.");
            }
            telemetry.addData("Target motor position (ticks): ", target_pos);
            telemetry.addData("Current motor position (ticks): ", motor_pos);
            telemetry.addData("Motor velocity (ticks/s): ", motor.getVelocity());
            telemetry.update();
        }
        motor.setPower(0);
    }
}
