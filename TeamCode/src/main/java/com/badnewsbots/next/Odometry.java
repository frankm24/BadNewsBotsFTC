package com.badnewsbots.next;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
Created by Frank on 7/20/2021.
Setup GitHub on 7/21/2021 using this video:
https://www.youtube.com/watch?v=iv8rSLsi1xo
For examples of what is going on here:
High level math from the Aluminum Cobblers 5436
--https://www.youtube.com/watch?v=-EJcpMsWJO4
--https://www.youtube.com/watch?v=wskMCVhVmoU
...and an example of applying it in Java by Wizards.exe #9794
--https://www.youtube.com/watch?v=cpdPtN4BDug&list=PLICNg-rquurYgWAQGhu6iC0At75vgqFJp&index=4
For more info on converting encoder ticks to inches of displacement:
https://renegaderobotics.org/vex-sensors-shaft-encoders/#auton
 */
public class Odometry {
    //Encoder value multipliers (a good idea used by Wizards.exe #9794, easy way to invert input value)
    private int leftEncoderMultiplier = 1, rightEncoderMultiplier = 1, horizontalEncoderMultiplier = 1;

    //Measured constants (NEEDS TO BE MEASURED, PLACEHOLDER)
    private double inchesPerTick = 0.5, distanceBetweenEncoders = 40;

    //Thread will wait threadSleepDelay milliseconds (currently unused)
    private int threadSleepDelay = 50; //unit = milliseconds

    //Define encoders (DcMotor class has encoder methods)
    private DcMotor left_encoder, right_encoder, horizontal_encoder;

    //Robot position and orientation values for calculations
    double globalXPosition = 0, globalYPosition = 0, globalHeading = 0;
    double leftEncoderPosition = 0, rightEncoderPosition = 0, horizontalEncoderPosition = 0;
    double prevLeftEncoderPosition = 0, prevRightEncoderPosition = 0, prevHorizontalEncoderPosition = 0;

    private boolean tracking = false; //Loop logic (see below)

    public void updatePosition() {
        //ALL angles and trig functions are in RADIANS NOT deg
        leftEncoderPosition = left_encoder.getCurrentPosition() * inchesPerTick;
        rightEncoderPosition = right_encoder.getCurrentPosition() * inchesPerTick;
        horizontalEncoderPosition = horizontal_encoder.getCurrentPosition() * inchesPerTick;
        //Variable names correspond to a visual diagram I drew after researching robotics odometry...
        double dL = leftEncoderPosition - prevLeftEncoderPosition; //All prev (and therefore change) will be zero if initial loop
        double dR = rightEncoderPosition - prevRightEncoderPosition;
        double dM = (dL + dR) / 2;
        double theta = (dR - dL) / distanceBetweenEncoders;
        double r = dM / theta;
        double a = globalHeading + theta/2;
        double Hypotenuse = (r * Math.sin(theta)) / (Math.cos(a));
        double dY = Math.sin(a) * Hypotenuse;
        double dX = Math.cos(a) * Hypotenuse;
        globalXPosition += dX;
        globalYPosition += dY;
        globalHeading += theta;
    }
    public double[] getPosition() {
        double[] position = {globalXPosition, globalYPosition, globalHeading};
        return position;
    }
    public void startTracking() {
        tracking = true;
        globalXPosition = 0; globalYPosition = 0; globalHeading = 0;

        while (tracking) {
            updatePosition();
        }
    }

    public void stopTracking() { tracking = false; }

    public void setEncoders(DcMotor l, DcMotor r, DcMotor h) {
        left_encoder = l;
        right_encoder = r;
        horizontal_encoder = h;
    }

}
