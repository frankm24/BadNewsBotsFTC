package badnewsbots.robots;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import badnewsbots.Object3D;

abstract class Robot {
    private OpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private MecanumDrive drive;

    abstract Pose2d getFieldPose();

}
