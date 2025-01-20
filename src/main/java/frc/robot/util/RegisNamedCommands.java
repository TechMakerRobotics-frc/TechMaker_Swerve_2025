package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;
// import frc.robot.commands.drive.AlignToTag;
import frc.robot.commands.leds.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.leds.Leds;
// import frc.robot.vision.VisionPose;

public class RegisNamedCommands {

    // private VisionPose visionPose;
    // private Drive drive;
    private Leds leds;

    /** Register commands in the pathplanner. */
    public RegisNamedCommands(
            // VisionPose visionPose,
            Drive drive,
            Leds leds) {
        // this.visionPose = visionPose;
        // this.drive = drive;
        this.leds = leds;

        // RegisterAlign();
        RegisterLeds();
    }


    /*private void RegisterAlign() {
      NamedCommands.registerCommand("Align", new AlignToTag(drive, visionPose, 20000));
    }*/

    private void RegisterLeds() {
        NamedCommands.registerCommand("Led Red", new LedRed(leds));
        NamedCommands.registerCommand("Led Green", new LedGreen(leds));
        NamedCommands.registerCommand("Led Blue", new LedBlue(leds));
        NamedCommands.registerCommand("Led Yellow", new LedYellow(leds));
        NamedCommands.registerCommand("Led Cian", new LedCian(leds));
        NamedCommands.registerCommand("Led White", new LedWhite(leds));
        NamedCommands.registerCommand("Led Rainbow", new LedRainbow(leds));
        NamedCommands.registerCommand("Led Off", new LedOff(leds));
    }
}
