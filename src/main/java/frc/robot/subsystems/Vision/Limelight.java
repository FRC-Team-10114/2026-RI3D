package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Constants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.RobotStatus;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;

public class Limelight extends SubsystemBase {

    private final CommandSwerveDrivetrain drive;
    private final String limelightName;

    private final Pigeon2 gyro;

    private int tagId = -1;

    public static final double fieldLength;
    public static final double fieldWidth;
    public final RobotStatus robotStatus;

    static {
        AprilTagFieldLayout layout;
        try {
            // 自動載入當年度的預設場地 (例如 2026 場地)
            layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (Exception e) {
            // 萬一讀不到檔案 (極少發生)，給個預設值防止程式崩潰
            // 這裡可以填入規則書上的大約數值
            layout = null;
            e.printStackTrace();
        }

        if (layout != null) {
            // 從官方資料直接抓取精確數值
            fieldLength = layout.getFieldLength();
            fieldWidth = layout.getFieldWidth();
        } else {
            // Fallback (保底數值)
            fieldLength = 16.54;
            fieldWidth = 8.21;
        }
    }

    public Limelight(
            CommandSwerveDrivetrain drive,
            String limelightName, RobotStatus robotStatus) {
        this.drive = drive;
        this.limelightName = limelightName;
        this.gyro = drive.getPigeon2();
        this.robotStatus = robotStatus;
    }

    @Override
    public void periodic() {
        LimelightHelpers.SetRobotOrientation(
                this.limelightName,
                this.gyro.getYaw().getValueAsDouble(),
                this.gyro.getAngularVelocityZWorld().getValueAsDouble(),
                this.gyro.getPitch().getValueAsDouble(),
                this.gyro.getAngularVelocityYWorld().getValueAsDouble(),
                this.gyro.getRoll().getValueAsDouble(),
                this.gyro.getAngularVelocityXWorld().getValueAsDouble());

        // MegaTag2 result

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (mt2 == null)
            return;

        tagId = (int) LimelightHelpers.getFiducialID(limelightName);

        // Filterts

        if (mt2.tagCount == 0)
            return;

        // 機器人旋轉太快時 (大於 maxYawRate度/秒)，視覺會有殘影，不使用數據
        // (假設 drive.getGyroYawRate() 回傳 deg/s)
        if (Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > LimelightConstants.MAX_GYRO_RATE)
            return;

        // 檢查座標是否跑出場地外 (X: 0~16.54m, Y: 0~8.21m)
        if (mt2.pose.getX() < 0 || mt2.pose.getX() > fieldLength ||
                mt2.pose.getY() < 0 || mt2.pose.getY() > fieldWidth)
            return;

        // ---------------------------------------------------------
        // 4. 計算標準差 (Trust Level)
        // ---------------------------------------------------------
        double xyStds;
        double degStds;
        double avgDist = mt2.avgTagDist;

        if (robotStatus.NeedResetPose) {
            xyStds = 0.01; // 改成跟 PhotonVision 一樣強
            degStds = 0.01; // 角度也極度信任
            robotStatus.NeedResetPose = false;
        } else {

            if (mt2.tagCount >= 2) {
                // 多 Tag：非常信任
                xyStds = 0.5;
                degStds = 6.0;
            } else {
                // 單 Tag：信任度隨距離遞減 (距離越遠，標準差越大)
                // 這裡使用距離的平方來快速降低遠距離的權重
                xyStds = 1.0 * (avgDist * avgDist);
                degStds = 999.0; // 單 Tag 完全不信任 MT2 算出的角度，只用它的 X/Y
            }
        }

        // ---------------------------------------------------------
        // 5. 送入 Drive Subsystem
        // ---------------------------------------------------------
        // 這裡需要你的 Drive 支援接收標準差 (Vector<N3>)
        drive.addVisionMeasurement(
                mt2.pose, // 視覺算出的 Pose2d
                mt2.timestampSeconds, // 這是正確的拍攝時間 (Latency Compensated)
                VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));

        Logger.recordOutput("Vision/Limelight/Pose/" + this.limelightName, mt2.pose);
        Logger.recordOutput("Vision/Limelight/TagID/" + this.limelightName, tagId);
    }

    // 提供給外部使用的 Getter
    public int getTagId() {
        return tagId;
    }
}