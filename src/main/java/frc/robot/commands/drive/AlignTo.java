package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class AlignTo extends Command {
  private final Drive drive;
  private final Pose2d targetPose;
  private final PIDController thetaController;

  public AlignTo(Drive drive, double x, double y) {
    this.drive = drive;
    this.targetPose = new Pose2d(x, y, new Rotation2d());
    this.thetaController = new PIDController(1.0, 0.0, 0.0);
    this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    this.thetaController.setTolerance(0.05);
    addRequirements(drive);
  }

  public AlignTo(Drive drive, Pose2d pose) {
    this.drive = drive;
    this.targetPose = pose;
    this.thetaController = new PIDController(1.0, 0.0, 0.0);
    this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    this.thetaController.setTolerance(0.05);
    addRequirements(drive);
  }

  public AlignTo(Drive drive, Translation2d translation) {
    this.drive = drive;
    this.targetPose = new Pose2d(translation, new Rotation2d());
    this.thetaController = new PIDController(1.0, 0.0, 0.0);
    this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    this.thetaController.setTolerance(0.05);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    thetaController.reset();
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
    Rotation2d currentRotation = currentPose.getRotation();
    double desiredTheta =
        Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX());
    double rotationSpeed = thetaController.calculate(currentRotation.getRadians(), desiredTheta);
    drive.runVelocity(new ChassisSpeeds(0, 0, rotationSpeed));
    ;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return thetaController.atSetpoint();
  }
}