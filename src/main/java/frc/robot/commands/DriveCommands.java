// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  

  // Maximum Allow wheel speed, in meters per second
  // SDS Mk4i wheels are 4" in diameter; to allow 2 revolutions per second, we calculate the circumference in inches, and convert to meters. 
  // Then multiply by 2 to represent the distance of 2 wheel revolutions
  final static double MAX_WHEEL_SPEED_METERS = Units.inchesToMeters(2 * Math.PI * 4.0) * 2;
  final static InterpolatingDoubleTreeMap wheelSpeedInterpolator = new InterpolatingDoubleTreeMap();

  // Maximum Allowed module turning speed, in radians per second
  // Set to 2pi = 1 revolution per second
  final static double MAX_TURN_SPEED_RADIANS = 2 * Math.PI;
  final static InterpolatingDoubleTreeMap turnSpeedInterpolator = new InterpolatingDoubleTreeMap();

  private static boolean interpolationInitialized = false;
  private static Rotation2d currentTurnAngle = new Rotation2d();

    public static void initializeInterpolationTreeMaps() {
        wheelSpeedInterpolator.put(-1.0, -MAX_WHEEL_SPEED_METERS);
        wheelSpeedInterpolator.put(0.0, 0.0);
        wheelSpeedInterpolator.put(1.0, MAX_WHEEL_SPEED_METERS);

        turnSpeedInterpolator.put(-1.0, -MAX_TURN_SPEED_RADIANS);
        turnSpeedInterpolator.put(0.0, 0.0);
        turnSpeedInterpolator.put(1.0, MAX_TURN_SPEED_RADIANS);

        interpolationInitialized = true;
    }


  public static Command singleModuleDrive(Drive drive,
      DoubleSupplier speedSupplier,
      DoubleSupplier turnSupplier) {
        if (!interpolationInitialized) {
            initializeInterpolationTreeMaps();
        }
        return Commands.run(() -> {

            // Raw Wheel and Turning speeds, deadbanded and clamped in the range [-1, 1]
            double rawSpeed = MathUtil.clamp(MathUtil.applyDeadband(speedSupplier.getAsDouble(), DEADBAND), -1, 1);
            double rawTurn = MathUtil.clamp(MathUtil.applyDeadband(turnSupplier.getAsDouble(), DEADBAND), -1, 1);

            double interpolatedSpeed = wheelSpeedInterpolator.get(rawSpeed);
            double interpolatedTurn = turnSpeedInterpolator.get(rawTurn);

            /*
             * Using Rotation2D for this is not exactly what I intended to do but at the moment I am too tired to rework 
             * the code I've written so far. What I *want* to do is figure out how to make the module rotate at a given 
             * speed (interpolated from the controller input), but this will have to do for now.
             */
            SwerveModuleState state = new SwerveModuleState(interpolatedSpeed, new Rotation2d(interpolatedTurn));

            drive.runSingleModule(state);
        }, drive);
      }  
}
