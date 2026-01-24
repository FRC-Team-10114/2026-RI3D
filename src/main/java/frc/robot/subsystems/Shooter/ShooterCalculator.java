// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.robot.util.AllianceFlipUtil;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.siteConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;

public class ShooterCalculator {
    private final CommandSwerveDrivetrain drive;
    private final InterpolatingTreeMap<Double, Rotation2d> hoodMap;
    private final InterpolatingTreeMap<Double, AngularVelocity> rollMap;
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
    private final double phaseDelay = 0.03;
    public static Transform3d robotToTurret = new Transform3d(0.2, 0.0, 0.44, Rotation3d.kZero);

    public ShooterCalculator(CommandSwerveDrivetrain drive) {
        this.drive = drive;
        hoodMap = new InterpolatingTreeMap<>(
                InverseInterpolator.forDouble(),
                Rotation2d::interpolate);
        rollMap = new InterpolatingTreeMap<>(
                InverseInterpolator.forDouble(),
                (start, end, t) -> {
                    // 邏輯：拆成 double (RPM) -> 算數學 -> 包回 Unit
                    double startVal = start.in(RotationsPerSecond);
                    double endVal = end.in(RotationsPerSecond);
                    double interpolated = MathUtil.interpolate(startVal, endVal, t);
                    return RotationsPerSecond.of(interpolated);
                });
        hoodMap.put(2.0, Rotation2d.fromDegrees(15.0));

        rollMap.put(2.0, RotationsPerSecond.of(3000));

        timeOfFlightMap.put(5.68, 1.16);
        timeOfFlightMap.put(4.55, 1.12);
        timeOfFlightMap.put(3.15, 1.11);
        timeOfFlightMap.put(1.88, 1.09);
        timeOfFlightMap.put(1.38, 0.90);
    }

    public void predictedcoordinates() {
        Pose2d estimatedPose = drive.getPose2d();
        ChassisSpeeds robotRelativeVelocity = drive.getChassisSpeeds();

        estimatedPose = estimatedPose.exp(new Twist2d(robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

        Pose2d turretPosition = estimatedPose.transformBy(
                new Transform2d(
                        robotToTurret.getTranslation().toTranslation2d(),
                        robotToTurret.getRotation().toRotation2d()));
        Translation2d target = AllianceFlipUtil.apply(siteConstants.topCenterPoint.toTranslation2d());
        double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

        ChassisSpeeds robotVelocity = drive.getFieldVelocity();
        double robotAngle = estimatedPose.getRotation().getRadians();
        double turretVelocityX = robotVelocity.vxMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                        * (robotToTurret.getY() * Math.cos(robotAngle)
                                - robotToTurret.getX() * Math.sin(robotAngle));
        double turretVelocityY = robotVelocity.vyMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                        * (robotToTurret.getX() * Math.cos(robotAngle)
                                - robotToTurret.getY() * Math.sin(robotAngle));

        double timeOfFlight;
        Pose2d lookaheadPose = turretPosition;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;
        for (int i = 0; i < 20; i++) {
            timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;
            lookaheadPose = new Pose2d(
                    turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    turretPosition.getRotation());
            lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }
        Logger.recordOutput("predictedcoordinates",lookaheadTurretToTargetDistance);
    }
}