package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A utility class for interacting with an arcade-style joystick. This class maps the buttons of the
 * joystick to `Trigger` objects and provides access to each button through methods and an array.
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
    buttons = new Trigger[16];

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
  public Trigger getButton(int index) {
    return buttons[index - 1];
  }

  /**
   * @return the Trigger object for button A
   */
  public Trigger a() {
    return getButton(1);
  }

  /**
   * @return the Trigger object for button B
   */
  public Trigger b() {
    return getButton(2);
  }

  /**
   * @return the Trigger object for button C
   */
  public Trigger c() {
    return getButton(3);
  }

  /**
   * @return the Trigger object for button D
   */
  public Trigger d() {
    return getButton(4);
  }

  /**
   * @return the Trigger object for button E
   */
  public Trigger e() {
    return getButton(5);
  }

  /**
   * @return the Trigger object for button F
   */
  public Trigger f() {
    return getButton(6);
  }

  /**
   * @return the Trigger object for button G
   */
  public Trigger g() {
    return getButton(7);
  }

  /**
   * @return the Trigger object for button H
   */
  public Trigger h() {
    return getButton(8);
  }

  /**
   * @return the Trigger object for button I
   */
  public Trigger i() {
    return getButton(9);
  }

  /**
   * @return the Trigger object for button J)
   */
  public Trigger j() {
    return getButton(10);
  }

  /**
   * @return the Trigger object for button K)
   */
  public Trigger k() {
    return getButton(11);
  }

  /**
   * @return the Trigger object for button L)
   */
  public Trigger l() {
    return getButton(12);
  }

  /**
   * @return the Trigger object for button M)
   */
  public Trigger m() {
    return getButton(13);
  }

  /**
   * @return the Trigger object for button N)
   */
  public Trigger n() {
    return getButton(14);
  }

  /**
   * @return the Trigger object for button O)
   */
  public Trigger o() {
    return getButton(15);
  }

  /**
   * @return the Trigger object for button P)
   */
  public Trigger p() {
    return getButton(16);
  }
}
