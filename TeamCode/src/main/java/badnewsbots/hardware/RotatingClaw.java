package badnewsbots.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Hashtable;
import java.util.Objects;

public class RotatingClaw {
    public enum Side {
        FRONT,
        BACK,
    }

    public enum SlideHeight {
        HIGH_GOAL,
        MID_GOAL,
        LOW_GOAL,
        INITIAL_GRAB,
        SLIGHTLY_UP,
        BOTTOM,
        OTHER
    }

    private final Hashtable<SlideHeight, Float> slideHeights = new Hashtable<>();

    public enum GripperState {
        GRIPPED,
        RELEASED
    }
    private final Telemetry telemetry;
    private final Servo gripperServo;
    private final Servo wristServo;
    private LimitSwitch zeroLimitSwitch;
    //private final LinearSlide linearSlide1;
    private final LinearSlide linearSlide2;
    private final double frontPosition = 0.92; // normal is 1
    private final double backPosition = 0.4; // normal is .4
    private final double grippingPosition = 0.35;
    private final double openPosition = 0;
    private final int slideIncrementAmount = 1;

    private Side currentSide = Side.FRONT;
    private GripperState currentGripperState = GripperState.RELEASED;
    private SlideHeight currentSlideHeight = SlideHeight.OTHER;

    public RotatingClaw(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.gripperServo = hardwareMap.get(Servo.class, "gripper");
        this.wristServo = hardwareMap.get(Servo.class, "wrist");
        //DcMotorEx slideMotor1 = hardwareMap.get(DcMotorEx.class, "linearSlide1");
        DcMotorEx slideMotor2 = hardwareMap.get(DcMotorEx.class, "linearSlide2");
        // TODO: Test and maybe change length to 38 1/8 inches instead if values inaccurate
        // 97.3cm long
        //this.linearSlide1 = new LinearSlide(slideMotor1, 38.30709f, 3251, false);
        this.linearSlide2 = new LinearSlide(slideMotor2, 38.30709f, 3251, false);
        //RevTouchSensor touchSensor =  hardwareMap.get(RevTouchSensor.class, "slide2ZeroLimit");
        //this.zeroLimitSwitch = new LimitSwitch(touchSensor, telemetry);

        slideHeights.put(SlideHeight.HIGH_GOAL, 35f);
        slideHeights.put(SlideHeight.MID_GOAL, 25f);
        slideHeights.put(SlideHeight.LOW_GOAL, 15f);
        slideHeights.put(SlideHeight.INITIAL_GRAB, 8f);
        //slideHeights.put(SlideHeight.SLIGHTLY_UP, 8f);
        slideHeights.put(SlideHeight.BOTTOM, 0f);
    }

    public void update() {
        zeroLimitSwitch.update();
        if (zeroLimitSwitch.isDown()) {
            //linearSlide1.zero();
            linearSlide2.zero();
        }
    }

    public void initialGrab() {
        grip();
        moveSlidesToPresetHeight(SlideHeight.INITIAL_GRAB);
        rotateToOtherSide();
    }

    public void letGo() {
        //linearSlide.decrementPositionTicks(100);
        release();
        rotateToOtherSide();
        moveSlidesToPresetHeight(SlideHeight.BOTTOM);
    }

    public void grip() {gripperServo.setPosition(grippingPosition);}
    public void release() {gripperServo.setPosition(openPosition);}

    // Grip if ungripped, ungrip if gripped
    public void toggleGrip() {
        if (currentGripperState == GripperState.GRIPPED) {
            release();
            currentGripperState = GripperState.RELEASED;
        }
        else if (currentGripperState == GripperState.RELEASED) {
            currentGripperState = GripperState.GRIPPED;
            grip();
        }
    }

    public void rotateToSide(Side side) {
        currentSide = side;
        if (side == Side.FRONT) {
            wristServo.setPosition(frontPosition);
        }
        else if (side == Side.BACK) {
            wristServo.setPosition(backPosition);
        }
    }

    public void rotateToOtherSide() {
        if (currentSide == Side.FRONT) {
            rotateToSide(Side.BACK);
        }
        else if (currentSide == Side.BACK) {rotateToSide(Side.FRONT);}
    }

    public void moveSlidesToPresetHeight(SlideHeight height) {
        currentSlideHeight = height;
        float heightInches = slideHeights.get(height);
        //linearSlide1.moveToPositionInches(heightInches);
        linearSlide2.moveToPositionInches(heightInches);
    }

    public void incrementSlidePositions() {
        currentSlideHeight = SlideHeight.OTHER;
        //linearSlide1.incrementPositionTicks(slideIncrementAmount);
        linearSlide2.incrementPositionTicks(slideIncrementAmount);
    }

    public void decrementSlidePositions() {
        currentSlideHeight = SlideHeight.OTHER;
        //linearSlide1.decrementPositionTicks(slideIncrementAmount);
        linearSlide2.decrementPositionTicks(slideIncrementAmount);
    }
    public int getSlide2PosTicks() {
        return linearSlide2.getMotor().getCurrentPosition();
    }
    public SlideHeight getCurrentSlideHeight() {return currentSlideHeight;}
    public GripperState getCurrentGripperState() {return currentGripperState;}
}
