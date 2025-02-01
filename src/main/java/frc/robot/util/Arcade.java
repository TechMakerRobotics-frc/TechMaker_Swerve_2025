package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Utility class for interacting with an arcade-style joystick.
 * This class maps joystick buttons to `Trigger` objects and provides
 * access to each button through dedicated methods.
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
     * Retrieves a button as a Trigger object by its index.
     *
     * @param index the 1-based index of the button
     * @return the corresponding Trigger object
     */
    private Trigger getButton(int index) {
        return buttons[index - 1];
    }

    /**
     * Gets the Trigger object for button 1.
     * @return the Trigger for reef position 1
     */
    public Trigger reefPosition1() { 
        return getButton(1); 
    }

    /**
     * Gets the Trigger object for button 2.
     * @return the Trigger for reef position 2
     */
    public Trigger reefPosition2() { 
        return getButton(2); 
    }

    /**
     * Gets the Trigger object for button 3.
     * @return the Trigger for reef position 3
     */
    public Trigger reefPosition3() { 
        return getButton(3); 
    }

    /**
     * Gets the Trigger object for button 4.
     * @return the Trigger for reef position 4
     */
    public Trigger reefPosition4() { 
        return getButton(4); 
    }

    /**
     * Gets the Trigger object for button 5.
     * @return the Trigger for reef position 5
     */
    public Trigger reefPosition5() { 
        return getButton(5); 
    }

    /**
     * Gets the Trigger object for button 6.
     * @return the Trigger for reef position 6
     */
    public Trigger reefPosition6() { 
        return getButton(6); 
    }

    /**
     * Gets the Trigger object for button 7.
     * @return the Trigger for reef level A1
     */
    public Trigger reefLevelA1() { 
        return getButton(7); 
    }

    /**
     * Gets the Trigger object for button 8.
     * @return the Trigger for reef level A2
     */
    public Trigger reefLevelA2() { 
        return getButton(8); 
    }

    /**
     * Gets the Trigger object for button 9.
     * @return the Trigger for reef level A3
     */
    public Trigger reefLevelA3() { 
        return getButton(9); 
    }

    /**
     * Gets the Trigger object for button 10.
     * @return the Trigger for reef level A4
     */
    public Trigger reefLevelA4() { 
        return getButton(10); 
    }

    /**
     * Gets the Trigger object for button 11.
     * @return the Trigger for reef level B1
     */
    public Trigger reefLevelB1() { 
        return getButton(11); 
    }

    /**
     * Gets the Trigger object for button 12.
     * @return the Trigger for reef level B2
     */
    public Trigger reefLevelB2() { 
        return getButton(12); 
    }

    /**
     * Gets the Trigger object for button 13.
     * @return the Trigger for reef level B3
     */
    public Trigger reefLevelB3() { 
        return getButton(13); 
    }

    /**
     * Gets the Trigger object for button 14.
     * @return the Trigger for reef level B4
     */
    public Trigger reefLevelB4() { 
        return getButton(14); 
    }
}
