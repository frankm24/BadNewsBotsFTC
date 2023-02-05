package badnewsbots.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LinearSlide {
    private final DcMotorEx motor;
    private final float LENGTH_IN;
    private final int LENGTH_TICKS;
    private final float MOTOR_TICKS_PER_INCH;
    private final int encoderDirection;
    private int adjustedZeroPosition = 0;
    private int adjustedMaxPosition;

    public LinearSlide(DcMotorEx motor, float lengthIn, int lengthTicks, boolean negateTicks) {
        this.LENGTH_IN = lengthIn;
        this.LENGTH_TICKS = lengthTicks;
        this.MOTOR_TICKS_PER_INCH = LENGTH_TICKS / LENGTH_IN;
        this.motor = motor;
        if (negateTicks) {
            encoderDirection = -1;}
        else {
            encoderDirection = 1;}
        adjustedZeroPosition = motor.getCurrentPosition();
        zero();
        //motor.setTargetPositionTolerance(0); // test if works once it works normally
    }

    public void zero() {
        adjustedZeroPosition = motor.getCurrentPosition();
        adjustedMaxPosition = LENGTH_TICKS + adjustedZeroPosition;
    }

    // Tell the slide to move to a given position in ticks
    public void moveToPositionTicks(int positionTicks) {
        positionTicks = Math.max(adjustedZeroPosition, Math.min(positionTicks, adjustedMaxPosition));
        motor.setTargetPosition(encoderDirection * positionTicks);
        motor.setPower(0.5);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void incrementPositionTicks(int amount) {
        int currentPosition = motor.getCurrentPosition();
        int requestedPosition = currentPosition + amount;
        int targetPosition = Math.min(requestedPosition, adjustedMaxPosition);
        motor.setTargetPosition(encoderDirection * targetPosition);
    }

    public void decrementPositionTicks(int amount) {
        int currentPosition = motor.getCurrentPosition();
        int requestedPosition = currentPosition - amount;
        int targetPosition = Math.max(adjustedZeroPosition, requestedPosition);
        motor.setTargetPosition(encoderDirection * targetPosition);
    }

    public void moveToPositionInches(float positionInches) {
        positionInches = Math.max(0, Math.min(positionInches, LENGTH_IN));
        int targetPosition = Math.round(positionInches * MOTOR_TICKS_PER_INCH * encoderDirection + adjustedZeroPosition);
        moveToPositionTicks(targetPosition);
    }

    public DcMotorEx getMotor() {
        return motor;
    }
}
