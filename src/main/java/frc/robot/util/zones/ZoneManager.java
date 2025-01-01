package frc.robot.util.zones;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;
import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import org.littletonrobotics.junction.AutoLogOutput;

public class ZoneManager {
    private Map<String, List<ZoneCircle>> zones;
    private final Drive drive;
    private Pose2d currentPose;

    private final ScheduledExecutorService scheduler;

    public static class ZoneCircle {
        public double center_x;
        public double center_y;
        public double radius;

        public Translation2d getCenter() {
            return new Translation2d(center_x, center_y);
        }
    }

    public ZoneManager(Drive drive) {

        this.drive = drive;

        ObjectMapper mapper = new ObjectMapper();
        File jsonFile = new File("src/main/java/frc/robot/util/zones/zones.json");

        try {
            ZoneData zoneData = mapper.readValue(jsonFile, ZoneData.class);

            this.zones = new java.util.HashMap<>();
            for (Map.Entry<String, List<ZoneCircle>> entry : zoneData.zones.entrySet()) {
                this.zones.put(entry.getKey(), entry.getValue());
            }
        } catch (IOException e) {
        }

        scheduler = Executors.newScheduledThreadPool(1);
        scheduler.scheduleAtFixedRate(this::periodic, 0, 20, TimeUnit.MILLISECONDS);
    }

    private void periodic() {
        currentPose = drive.getPose();
    }

    public List<ZoneCircle> getZoneCircles(String zoneName) {
        if (zones.containsKey(zoneName)) {
            return zones.get(zoneName);
        } else {
            throw new IllegalArgumentException("Zona não encontrada: " + zoneName);
        }
    }

    @AutoLogOutput(key = "ZoneManager/currentPose")
    public Pose2d getCurrentPose() {
        return currentPose;
    }

    public void stop() {
        scheduler.shutdown();
    }

    public static class ZoneData {
        public Map<String, List<ZoneCircle>> zones;
    }

    @AutoLogOutput(key = "ZoneManager/currentZone")
    public String getCurrentZone() {
        Translation2d robotPosition = currentPose.getTranslation();

        for (Map.Entry<String, List<ZoneCircle>> entry : zones.entrySet()) {
            String zoneName = entry.getKey();
            List<ZoneCircle> zoneCircles = entry.getValue();

            for (ZoneCircle circle : zoneCircles) {
                Translation2d center = circle.getCenter();
                double distance = robotPosition.getDistance(center);

                if (distance <= circle.radius) {
                    return zoneName;
                }
            }
        }

        return "Is not in a zone";
    }
}
