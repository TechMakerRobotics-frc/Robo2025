package frc.robot.util.zones;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import org.littletonrobotics.junction.AutoLogOutput;

public class ZoneManager {
  private Map<String, List<Pose2d>> zones;
  private final String zoneName;
  private final Drive drive;
  private Pose2d closestPose = new Pose2d();
  private Pose2d currentPose;
  private Pose2d[] poses;

  private final ScheduledExecutorService scheduler;

  public static class PoseData {
    public double x;
    public double y;
    public double heading;

    public Pose2d toPose2d() {
      return new Pose2d(x, y, Rotation2d.fromDegrees(heading));
    }
  }

  public ZoneManager(Drive drive, String zoneName) throws IOException {
    this.drive = drive;
    this.zoneName = zoneName;

    ObjectMapper mapper = new ObjectMapper();
    File jsonFile = new File("src/main/java/frc/robot/util/zones/zones.json");

    // Verifique se o arquivo existe
    if (!jsonFile.exists()) {
      throw new IOException(
          "Arquivo zones.json não encontrado no diretório: " + jsonFile.getAbsolutePath());
    }

    try {
      ZoneData zoneData = mapper.readValue(jsonFile, ZoneData.class);

      if (zoneData.zones == null || zoneData.zones.isEmpty()) {
        throw new IOException("Nenhuma zona encontrada no arquivo JSON.");
      }

      this.zones = new java.util.HashMap<>();
      for (Map.Entry<String, List<PoseData>> entry : zoneData.zones.entrySet()) {
        List<Pose2d> pose2dList = new ArrayList<>();
        for (PoseData poseData : entry.getValue()) {
          pose2dList.add(poseData.toPose2d());
        }
        this.zones.put(entry.getKey(), pose2dList);
      }

      System.out.println("Zonas carregadas com sucesso: " + this.zones.size() + " zonas");
    } catch (IOException e) {
      throw new IOException("Erro ao ler o arquivo JSON: " + e.getMessage(), e);
    }

    scheduler = Executors.newScheduledThreadPool(1);
    scheduler.scheduleAtFixedRate(this::periodic, 0, 80, TimeUnit.MILLISECONDS);
  }

  private void periodic() {
    currentPose = drive.getPose();
    calculateClosestPose();
  }

  public List<Pose2d> getZonePoses(String zoneName) {
    if (zones.containsKey(zoneName)) {
      return zones.get(zoneName);
    } else {
      throw new IllegalArgumentException("Zona não encontrada: " + zoneName);
    }
  }

  private void calculateClosestPose() {
    List<Pose2d> zonePoses = getZonePoses(zoneName);
    poses = zonePoses.toArray(new Pose2d[0]);

    closestPose =
        zonePoses.stream()
            .min(
                (p1, p2) ->
                    Double.compare(
                        p1.getTranslation().getDistance(currentPose.getTranslation()),
                        p2.getTranslation().getDistance(currentPose.getTranslation())))
            .orElseThrow(() -> new IllegalArgumentException("Lista de poses está vazia!"));
  }

  @AutoLogOutput(key = "ZoneManager/closestPose")
  public Pose2d getClosestPoseZone() {
    return closestPose;
  }

  @AutoLogOutput(key = "ZoneManager/currentPose")
  public Pose2d getCurrentPose() {
    return currentPose;
  }

  @AutoLogOutput(key = "ZoneManager/ListPoses")
  public Pose2d[] getCurrentPoses() {
    return poses;
  }

  public void stop() {
    scheduler.shutdown();
  }

  public static class ZoneData {
    public Map<String, List<PoseData>> zones;
  }
}
