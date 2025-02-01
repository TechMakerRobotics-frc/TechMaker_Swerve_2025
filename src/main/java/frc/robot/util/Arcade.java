package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A utility class for interacting with an arcade-style joystick.
 * This class maps the buttons of the joystick to `Trigger` objects
 * and provides access to each button through methods and an array.
 */
public class Arcade {
    private final Joystick joy;

    private final Trigger[] buttons;

    /**
     * Constructs an Arcade object for a joystick at the specified port.
     *
     * @param port the port number of the joystick
     */
    public Arcade(int port) {
        this.joy = new Joystick(port);
        buttons = new Trigger[14];

        for (int i = 0; i < buttons.length; i++) {
            buttons[i] = new JoystickButton(joy, i + 1);
        }
    }

    /**
     * Gets the button as a Trigger object by its index.
     *
     * @param index the 1-based index of the button
     * @return the corresponding Trigger object
     */
    private Trigger getButton(int index) {
        return buttons[index - 1];
    }

    /**
     * @return the Trigger object for button A
     */
    public Trigger reefPosition1() {
        return getButton(1);
    }

    /**
     * @return the Trigger object for button B
     */
    public Trigger reefPosition2() {
        return getButton(2);
    }

    /**
     * @return the Trigger object for button C
     */
    public Trigger reefPosition3() {
        return getButton(3);
    }

    /**
     * @return the Trigger object for button D
     */
    public Trigger reefPosition4() {
        return getButton(4);
    }

    /**
     * @return the Trigger object for button E
     */
    public Trigger reefPosition5() {
        return getButton(5);
    }

    /**
     * @return the Trigger object for button F
     */
    public Trigger reefPosition6() {
        return getButton(6);
    }

    /**
     * @return the Trigger object for button G
     */
    public Trigger reefLevelA1() {
        return getButton(7);
    }

    /**
     * @return the Trigger object for button H
     */
    public Trigger reefLevelA2() {
        return getButton(8);
    }

    /**
     * @return the Trigger object for button I
     */
    public Trigger reefLevelA3() {
        return getButton(9);
    }

    /**
     * @return the Trigger object for button J)
     */
    public Trigger reefLevelA4() {
        return getButton(10);
    }

    /**
     * @return the Trigger object for button K)
     */
    public Trigger reefLevelB1() {
        return getButton(11);
    }

    /**
     * @return the Trigger object for button L)
     */
    public Trigger reefLevelB2() {
        return getButton(12);
    }

    /**
     * @return the Trigger object for button M)
     */
    public Trigger reefLevelB3() {
        return getButton(13);
    }

    /**
     * @return the Trigger object for button N)
     */
    public Trigger reefLevelB4() {
        return getButton(14);
    }
}
