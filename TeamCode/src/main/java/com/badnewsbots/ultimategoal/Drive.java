package com.badnewsbots.ultimategoal;

import com.badnewsbots.ultimategoal.Points;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Deprecated
public class Drive {
    private DcMotor back_left;
    private DcMotor front_left;
    private DcMotor back_right;
    private DcMotor front_right;

    public void setMotors(DcMotor bl, DcMotor fl, DcMotor br, DcMotor fr) {
        back_left = bl;
        front_left = fl;
        back_right = br;
        front_right = fr;
    }

    public double getPowerFromMagnitude(double stickInputX, double stickInputY) {
        Points points = new Points();
        double magnitude = points.distanceFormula(0, 0, stickInputX, stickInputY);
        return magnitude / 1.2;
    }

    public void setMotorsToZero() {
        back_left.setPower(0);
        front_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);
    }
    public void forward(double power) {
        back_left.setPower(power);
        front_left.setPower(power);
        front_right.setPower(power);
        back_right.setPower(power);
    }
    public void backward(double power) {
        back_left.setPower(-power);
        front_left.setPower(-power);
        front_right.setPower(-power);
        back_right.setPower(-power);
    }
    public void left(double power) {
        back_left.setPower(power);
        front_left.setPower(-power);
        front_right.setPower(power);
        back_right.setPower(-power);
    }
    public void right(double power) {
        back_left.setPower(-power);
        front_left.setPower(power);
        front_right.setPower(-power);
        back_right.setPower(power);
    }
    public void forwardRight(double power) {
        front_left.setPower(power);
        back_right.setPower(power);
        front_right.setPower(0);
        back_left.setPower(0);
    }
    public void forwardLeft(double power) {
        front_right.setPower(power);
        back_left.setPower(power);
        front_left.setPower(0);
        back_right.setPower(0);
    }
    public void backLeft(double power) {
        front_left.setPower(-power);
        back_right.setPower(-power);
        front_right.setPower(0);
        front_left.setPower(0);
    }
    public void backRight(double power) {
        front_right.setPower(-power);
        back_left.setPower(-power);
        front_left.setPower(0);
        back_right.setPower(0);
    }
    public void turnLeft(double power) {
        front_left.setPower(-power);
        back_left.setPower(-power);
        front_right.setPower(power);
        back_right.setPower(power);
    }
    public void turnRight(double power) {
        front_left.setPower(power);
        back_left.setPower(power);
        front_right.setPower(-power);
        back_right.setPower(-power);
    }
}
