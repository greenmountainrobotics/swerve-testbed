package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.drive.SparkFlexOdometryThread;
import java.util.Queue;

public class GyroIONavX2 implements GyroIO {

  private final AHRS ahrs = new AHRS(SPI.Port.kMXP);
  private final Queue<Double> yawQueue;
  private double yaw;

  public GyroIONavX2() {
    ahrs.reset();

    yawQueue = SparkFlexOdometryThread.getInstance().registerSignal(() -> -ahrs.getYaw());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = ahrs.isConnected();

    inputs.yawPosition =
        Rotation2d.fromDegrees(-ahrs.getYaw()); // TODO: check if this is degrees or radians
    inputs.odometryYawPositions =
        yawQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
    yawQueue.clear();
  }
}