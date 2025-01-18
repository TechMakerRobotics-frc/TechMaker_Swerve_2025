package frc.robot.subsystems.drive;

import java.util.List;

import frc.robot.util.Circle2d;

public class DriveConstants {

    public static class ZoneLocates {
        public static final Circle2d blueLeftStation = new Circle2d(1.100, 7.124, 2.6, Zones.BLUE_LEFT_STATION);
        public static final Circle2d blueRightStation = new Circle2d(1.100, 0.974, 2.6, Zones.BLUE_RIGHT_STATION);
        public static final Circle2d redLeftStation = new Circle2d(16.697198, 0.974, 2.6, Zones.RED_LEFT_STATION);
        public static final Circle2d redRightStation = new Circle2d(16.411, 6.968, 2.6, Zones.RED_RIGHT_STATION);
        public static final Circle2d blueProcessor = new Circle2d(6.341, 0.471, 1, Zones.BLUE_PROCESSOR);
        public static final Circle2d redProcessor = new Circle2d(11.540, 7.651, 1, Zones.RED_PROCESSOR);

        // Lista contendo todas as zonas
        public static final List<Circle2d> zones = List.of(
            blueLeftStation,
            blueRightStation,
            redLeftStation,
            redRightStation,
            blueProcessor,
            redProcessor
        );

        public static enum Zones {
            NOT_ZONE,
            BLUE_LEFT_STATION,
            BLUE_RIGHT_STATION,
            RED_LEFT_STATION,
            RED_RIGHT_STATION,
            BLUE_PROCESSOR,
            RED_PROCESSOR,
        }
    }
}
