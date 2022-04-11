package org.firstinspires.ftc.teamcode.slam;

import android.hardware.Sensor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class UltrasonicSensor {
    private ModernRoboticsI2cRangeSensor sensor;
    private Pose2d position;

    public UltrasonicSensor(ModernRoboticsI2cRangeSensor sensor, Pose2d position) {
        this.sensor = sensor;
        this.position = position;
    }

    public void setSensor(ModernRoboticsI2cRangeSensor sensor) {
        this.sensor = sensor;
    }

    public ModernRoboticsI2cRangeSensor getSensor() {
        return sensor;
    }

    public Pose2d getPosition() {
        return position;
    }

    public void setPosition(Pose2d position) {
        this.position = position;
    }
}
