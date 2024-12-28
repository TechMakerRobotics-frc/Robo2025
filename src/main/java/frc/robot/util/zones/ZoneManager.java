package frc.robot.util.zones;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;

public class ZoneManager extends SubsystemBase {
  private Map<String, List<Pose2d>> zones;
  private final String zoneName;
  private final Drive drive;
  private Pose2d closestPose = new Pose2d();
  private Pose2d currentPose;

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
      String errorMsg =
          "Arquivo zones.json não encontrado no diretório: " + jsonFile.getAbsolutePath();
      System.err.println(errorMsg);
      SmartDashboard.putString("ZoneManager/Status", errorMsg); // Log para o SmartDashboard
      throw new IOException(errorMsg);
    }

    try {
      ZoneData zoneData = mapper.readValue(jsonFile, ZoneData.class);

      // Verifique se há zonas carregadas do JSON
      if (zoneData.zones == null || zoneData.zones.isEmpty()) {
        String errorMsg = "Nenhuma zona encontrada no arquivo JSON.";
        System.err.println(errorMsg);
        SmartDashboard.putString("ZoneManager/Status", errorMsg); // Log para o SmartDashboard
        throw new IOException(errorMsg);
      }

      // Usando transformação para criar a lista de Pose2d
      this.zones = new java.util.HashMap<>();
      for (Map.Entry<String, List<PoseData>> entry : zoneData.zones.entrySet()) {
        List<Pose2d> pose2dList = new ArrayList<>();
        for (PoseData poseData : entry.getValue()) {
          Pose2d pose = poseData.toPose2d();
          pose2dList.add(pose);
        }
        this.zones.put(entry.getKey(), pose2dList);
      }

      String successMsg = "Zonas carregadas com sucesso: " + this.zones.size() + " zonas";
      System.out.println(successMsg);
      SmartDashboard.putString("ZoneManager/Status", successMsg); // Log para o SmartDashboard
      SmartDashboard.putNumber(
          "ZoneManager/TotalZones", this.zones.size()); // Log de número de zonas

    } catch (IOException e) {
      String errorMsg = "Erro ao ler o arquivo JSON: " + e.getMessage();
      System.err.println(errorMsg);
      SmartDashboard.putString("ZoneManager/Status", errorMsg); // Log para o SmartDashboard
      throw e; // Re-lança a exceção para indicar falha crítica no carregamento
    }
  }

  // Obtém as poses de uma zona específica
  public List<Pose2d> getZonePoses(String zoneName) {
    if (zones.containsKey(zoneName)) {
      return zones.get(zoneName);
    } else {
      String errorMsg = "Zona não encontrada: " + zoneName;
      System.err.println(errorMsg);
      SmartDashboard.putString("ZoneManager/Status", errorMsg); // Log para o SmartDashboard
      throw new IllegalArgumentException(errorMsg);
    }
  }

  @Override
  public void periodic() {
    currentPose = drive.getPose();
    calculateClosestPose();
  }

  public void calculateClosestPose() {
    List<Pose2d> zonePoses = getZonePoses(zoneName);

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

  public static class ZoneData {
    public Map<String, List<PoseData>> zones;
  }
}
