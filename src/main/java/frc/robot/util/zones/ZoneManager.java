package frc.robot.util.zones;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Zones;
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
    private String zoneName = "totalZones";
    private final Drive drive;
    private Pose2d closestPose = new Pose2d();
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

    public ZoneManager(Zones zone, Drive drive) throws IOException {
        switch (zone) {
            case BlueSpeakerZone:
                this.zoneName = "BlueSpeakerZone";
            case RedSpeakerZone:
                this.zoneName = "RedSpeakerZone";
                break;
        }
        this.drive = drive;

        ObjectMapper mapper = new ObjectMapper();
        File jsonFile = new File("src/main/java/frc/robot/util/zones/zones.json");

        // Verifique se o arquivo existe
        if (!jsonFile.exists()) {
            throw new IOException("Arquivo zones.json não encontrado no diretório: " + jsonFile.getAbsolutePath());
        }

        try {
            ZoneData zoneData = mapper.readValue(jsonFile, ZoneData.class);

            if (zoneData.zones == null || zoneData.zones.isEmpty()) {
                throw new IOException("Nenhuma zona encontrada no arquivo JSON.");
            }

            this.zones = new java.util.HashMap<>();
            for (Map.Entry<String, List<ZoneCircle>> entry : zoneData.zones.entrySet()) {
                this.zones.put(entry.getKey(), entry.getValue());
            }
        } catch (IOException e) {
            throw new IOException("Erro ao ler o arquivo JSON: " + e.getMessage(), e);
        }

        scheduler = Executors.newScheduledThreadPool(1);
        scheduler.scheduleAtFixedRate(this::periodic, 0, 20, TimeUnit.MILLISECONDS);
    }

    private void periodic() {
        currentPose = drive.getPose();
        calculateClosestPose();
    }

    public List<ZoneCircle> getZoneCircles(String zoneName) {
        if (zones.containsKey(zoneName)) {
            return zones.get(zoneName);
        } else {
            throw new IllegalArgumentException("Zona não encontrada: " + zoneName);
        }
    }

    private void calculateClosestPose() {
        if (!zoneName.equalsIgnoreCase("totalZones")) {
            List<ZoneCircle> zoneCircles = getZoneCircles(zoneName);

            closestPose = zoneCircles.stream()
                    .map(this::findClosestPointInCircle)
                    .min((p1, p2) -> Double.compare(
                            p1.getTranslation().getDistance(currentPose.getTranslation()),
                            p2.getTranslation().getDistance(currentPose.getTranslation())))
                    .orElseThrow(() -> new IllegalArgumentException("Lista de zonas está vazia!"));
        } else {
            closestPose = currentPose;
        }
    }

    private Pose2d findClosestPointInCircle(ZoneCircle circle) {
        Translation2d center = circle.getCenter();
        Translation2d robotTranslation = currentPose.getTranslation();

        // Vetor do centro para o robô
        Translation2d toRobot = robotTranslation.minus(center);
        double distanceToCenter = toRobot.getNorm();

        if (distanceToCenter <= circle.radius) {
            // Robô está dentro ou na borda do círculo
            return new Pose2d(robotTranslation, currentPose.getRotation());
        } else {
            // Robô está fora do círculo; ajusta para o ponto mais próximo na borda
            Translation2d closestPoint = center.plus(toRobot.times(circle.radius / distanceToCenter));
            return new Pose2d(closestPoint, currentPose.getRotation());
        }
    }

    @AutoLogOutput(key = "ZoneManager/closestPose")
    public Pose2d getClosestPose() {
        return closestPose;
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

        // Itera por todas as zonas e verifica se o robô está dentro de alguma
        for (Map.Entry<String, List<ZoneCircle>> entry : zones.entrySet()) {
            String zoneName = entry.getKey();
            List<ZoneCircle> zoneCircles = entry.getValue();

            for (ZoneCircle circle : zoneCircles) {
                Translation2d center = circle.getCenter();
                double distance = robotPosition.getDistance(center);

                if (distance <= circle.radius) {
                    return zoneName; // Retorna o nome da zona em que o robô está
                }
            }
        }

        return "Is not"; // Se o robô não estiver em nenhuma zona
    }
}
