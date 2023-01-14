package badnewsbots.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LinearSlide {
    private final DcMotorEx motor;
    private final float LENGTH_IN;
    private final int LENGTH_TICKS;
    private final float MOTOR_TICKS_PER_INCH;
    private final int encoderDirection;

    public LinearSlide(DcMotorEx motor, float lengthIn, int lengthTicks, boolean negateTicks) {
        this.LENGTH_IN = lengthIn;
        this.LENGTH_TICKS = lengthTicks;
        this.MOTOR_TICKS_PER_INCH = LENGTH_TICKS / LENGTH_IN;
        this.motor = motor;
        if (negateTicks) {
            encoderDirection = -1;}
        else {
            encoderDirection = 1;}
        //motor.setTargetPositionTolerance(0); // test if works once it works normally
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Tell the slide to move to a given position in ticks
    public void moveToPositionTicks(int positionTicks) {
        positionTicks = Math.max(0, Math.min(positionTicks, LENGTH_TICKS));
        motor.setTargetPosition(encoderDirection * positionTicks);
        motor.setPower(1);
    }

    public void incrementPositionTicks(int amount) {
        int currentPosition = motor.getCurrentPosition();
        int requestedPosition = currentPosition + amount;
        int targetPosition = Math.min(requestedPosition, LENGTH_TICKS);
        motor.setTargetPosition(encoderDirection * targetPosition);
    }

    public void decrementPositionTicks(int amount) {
        int currentPosition = motor.getCurrentPosition();
        int requestedPosition = currentPosition - amount;
        int targetPosition = Math.max(0, requestedPosition);
        motor.setTargetPosition(encoderDirection * targetPosition);
    }

    public void moveToPositionInches(float positionInches) {
        positionInches = Math.max(0, Math.min(positionInches, LENGTH_IN));
        int targetPosition = Math.round(positionInches * MOTOR_TICKS_PER_INCH * encoderDirection);
        moveToPositionTicks(targetPosition);
    }
}
