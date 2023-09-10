package badnewsbots.hardware;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimitSwitch {

    private final Telemetry telemetry;
    private final RevTouchSensor touchSensor;
    private boolean switchDownPrev = false;
    private boolean switchDown = false;
    private boolean switchPressed = false;
    private boolean switchReleased = false;

    public LimitSwitch(RevTouchSensor touchSensor, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.touchSensor = touchSensor;
    }

    public void update() {
        switchDownPrev = switchDown;
        switchDown = touchSensor.isPressed();
        switchPressed = !switchDownPrev && switchDown;
        switchReleased = switchDownPrev && !switchDown;
    }

    public boolean wasPressed() {return switchPressed;}
    public boolean wasReleased() {return switchReleased;}
    public boolean isDown() {return switchDown;}
}
