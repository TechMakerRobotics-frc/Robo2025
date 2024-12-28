package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.zones.ZoneManager;

public class DriveTo extends Command {

  private final Pose2d goalPose;
  private final double timeOut;
  private PathConstraints constraints = new PathConstraints(4.5, 2.5, 4.5, 2.5);
  private Command pathfollower;
  private Timer time = new Timer();
  private ZoneManager zoneManager;

  public DriveTo(Pose2d goalPose, double timeOut) {
    this.timeOut = timeOut;
    this.goalPose = goalPose;
  }

  public DriveTo(Translation2d goalTranslation, double timeOut) {
    this.timeOut = timeOut;
    this.goalPose = new Pose2d(goalTranslation, new Rotation2d());
  }

  public DriveTo(Translation2d goalTranslation, Rotation2d goalRotation, double timeOut) {
    this.timeOut = timeOut;
    this.goalPose = new Pose2d(goalTranslation, goalRotation);
  }

  public DriveTo(double x, double y, double timeOut) {
    this.timeOut = timeOut;
    this.goalPose = new Pose2d(x, y, new Rotation2d());
  }

  public DriveTo(double x, double y, double heading, double timeOut) {
    this.timeOut = timeOut;
    this.goalPose = new Pose2d(x, y, new Rotation2d(heading));
  }

  public DriveTo(Drive drive, ZoneManager zoneManager, double timeOut) {
    this.timeOut = timeOut;
    this.zoneManager = zoneManager;
    this.goalPose = this.zoneManager.getClosestPoseZone();
  }

  @Override
  public void initialize() {
    time.reset();
    time.start();
    pathfollower = AutoBuilder.pathfindToPose(goalPose, constraints);
    pathfollower.initialize();
  }

  @Override
  public void execute() {
    pathfollower.execute();
  }

  @Override
  public boolean isFinished() {
    return pathfollower == null || pathfollower.isFinished() || time.hasElapsed(timeOut);
  }
}
