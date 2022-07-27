package badnewsbots.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

/*
Written by Frank Murphy, 6/13/2022

The purpose of this WRAPPER class is to expand the functionality of the robotcore Gamepad class to automatically
handle button press and release detection rather than only detecting if a button is down. This works by storing the
values of each controller button from the previous frame, rather than using a debounce timer, which we used previously.
This class is designed to run synchronously with the main loop of an OpMode. Additional functionality will be added
as needed.

NOTE: ALL OF THESE VARIABLES ARE READ ONLY FROM OUTSIDE GAMEPADEX
NOTE 2: Not currently implementing the PS4-only values, such as touchpad because we do not use PS4 controllers.
 */
public class GamepadEx {
    // These values represent the current state of each control on the gamepad
    public boolean a;
    public boolean b;
    public boolean x;
    public boolean y;
    public boolean dpad_right;
    public boolean dpad_up;
    public boolean dpad_left;
    public boolean dpad_down;
    public boolean start;
    public boolean back;
    public boolean left_bumper;
    public boolean right_bumper;
    public float left_trigger;
    public boolean left_trigger_bool;
    public float right_trigger;
    public boolean right_trigger_bool;
    public float left_stick_x;
    public float left_stick_y;
    public boolean left_stick_button;
    public float right_stick_x;
    public float right_stick_y;
    public boolean right_stick_button;

    // These values represent the condition of each button in the previous frame
    boolean a_prev;
    boolean b_prev;
    boolean x_prev;
    boolean y_prev;
    boolean dpad_right_prev;
    boolean dpad_up_prev;
    boolean dpad_left_prev;
    boolean dpad_down_prev;
    boolean start_prev;
    boolean back_prev;
    boolean left_bumper_prev;
    boolean right_bumper_prev;
    boolean left_trigger_bool_prev;
    boolean right_trigger_bool_prev;
    boolean left_stick_button_prev;
    boolean right_stick_button_prev;

    // These values represent when a button has just been pressed.
    public boolean a_pressed;
    public boolean b_pressed;
    public boolean x_pressed;
    public boolean y_pressed;
    public boolean dpad_right_pressed;
    public boolean dpad_up_pressed;
    public boolean dpad_left_pressed;
    public boolean dpad_down_pressed;
    public boolean start_pressed;
    public boolean back_pressed;
    public boolean left_bumper_pressed;
    public boolean right_bumper_pressed;
    public boolean left_trigger_pressed;
    public boolean right_trigger_pressed;
    public boolean left_stick_button_pressed;
    public boolean right_stick_button_pressed;

    // These values represent when a button has just been released.
    public boolean a_released;
    public boolean b_released;
    public boolean x_released;
    public boolean y_released;
    public boolean dpad_right_released;
    public boolean dpad_up_released;
    public boolean dpad_left_released;
    public boolean dpad_down_released;
    public boolean start_released;
    public boolean back_released;
    public boolean left_bumper_released;
    public boolean right_bumper_released;
    public boolean left_trigger_released;
    public boolean right_trigger_released;
    public boolean left_stick_button_released;
    public boolean right_stick_button_released;

    // UNFORTUNATELY, FtcRobotController is programmed so that you CANNOT simply extend the gamepad
    // class, so I have made GamepadEx a "wrapper" class, which means a Gamepad from the OpMode will be
    // passed in when an instance of GamepadEx is constructed and used to get the gamepad's info and do
    // additional functions with. I wish it didn't have to be this way, but there is still benefit to making a
    // GamepadEx class because it allows us to create
    private final Gamepad gamepad;
    public GamepadEx(Gamepad gamepad) {
        this.gamepad = gamepad;
    }
    public Gamepad getGamepad() {
        return gamepad;
    }

    // Call this method at the START of the loop.
    public void update() {
        a = gamepad.a;
        b = gamepad.b;
        x = gamepad.x;
        y = gamepad.y;
        dpad_right = gamepad.dpad_right;
        dpad_up = gamepad.dpad_up;
        dpad_left = gamepad.dpad_left;
        dpad_down = gamepad.dpad_down;
        start = gamepad.start;
        back = gamepad.back;
        left_bumper = gamepad.left_bumper;
        right_bumper = gamepad.right_bumper;
        left_trigger = gamepad.left_trigger;
        left_trigger_bool = left_trigger > 0;
        right_trigger = gamepad.right_trigger;
        right_trigger_bool = right_trigger > 0;
        left_stick_x = gamepad.left_stick_x;
        left_stick_y = gamepad.left_stick_y;
        left_stick_button = gamepad.left_stick_button;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;
        right_stick_button = gamepad.right_stick_button;

        a_pressed = !a_prev && a;
        b_pressed = !b_prev && b;
        x_pressed = !x_prev && x;
        y_pressed = !a_prev && y;
        dpad_right_pressed = !dpad_right_prev && dpad_right;
        dpad_up_pressed = !dpad_up_prev && dpad_up;
        dpad_left_pressed = !dpad_left_prev && dpad_left;
        dpad_down_pressed = !dpad_down_prev && dpad_down;
        start_pressed = !start_prev && start;
        back_pressed = !back_prev && back;
        left_bumper_pressed = !left_bumper_prev && left_bumper;
        right_bumper_pressed = !right_bumper_prev && right_bumper;
        left_trigger_pressed = !left_trigger_bool_prev && left_trigger_bool;
        right_trigger_pressed = !right_trigger_bool_prev && right_trigger_bool;
        left_stick_button_pressed = !left_stick_button_prev && left_stick_button;
        right_stick_button_pressed = !right_stick_button_prev && right_stick_button;

        a_released = a_prev && !a;
        b_released = b_prev && !b;
        x_released = x_prev && !x;
        y_released = y_prev && !y;
        dpad_right_released = dpad_right_prev && !dpad_right;
        dpad_up_released = dpad_up_prev && !dpad_up;
        dpad_left_released = dpad_left_prev && !dpad_left;
        dpad_down_released = dpad_down_prev && !dpad_down;
        start_released = start_prev && !start;
        back_released = back_prev && !back;
        left_bumper_released = left_bumper_prev && !left_bumper;
        right_bumper_released = right_bumper_prev && !right_bumper;
        left_trigger_released = left_trigger_bool_prev && !left_trigger_bool;
        right_trigger_released = right_trigger_bool_prev && !right_trigger_bool;
        left_stick_button_released = left_stick_button_prev && !left_stick_button;
        right_stick_button_released = right_stick_button_prev && !right_stick_button;
    }
    // This method overwrites the data from the previous frame to the current values
    // Only call this method at the END of the loop, or when you do no longer need to access data from the previous frame.
    public void postUpdate() {
        a_prev = a;
        b_prev = b;
        x_prev = x;
        y_prev = y;
        dpad_right_prev = dpad_right;
        dpad_up_prev = dpad_up;
        dpad_left_prev = dpad_left;
        dpad_down_prev = dpad_down;
        start_prev = start;
        back_prev = back;
        left_bumper_prev = left_bumper;
        right_bumper_prev = right_bumper;
        left_trigger_bool_prev = left_trigger_bool;
        right_trigger_bool_prev = right_trigger_bool;
        left_stick_button_prev = left_stick_button;
        right_stick_button_prev = right_stick_button;
    }
}