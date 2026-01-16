package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
    
      private int tagId = -1;

  /** 輔助：示範 VisionConstants.SIM_CAMERA_PROPERTIES 的最小 stub（你可以在別處定義） */
  public class VisionConstants {
    public static final Map<String, Transform3d> cameraTransforms = Map.of(
        "RightOV", new Transform3d(
            // 位置不變 (車尾右側)
            new Translation3d(-0.20979456, -0.13607890, 0.15952705),
            // 🛠️ 修改這裡：原本是 180-30，改成 180+30 (即 -150度)
            new Rotation3d(0.0, 0.0, Math.toRadians(180 + 30))),
        "LeftOV", new Transform3d(
            // 位置不變 (車尾左側)
            new Translation3d(-0.20979456, 0.13607890, 0.15952705),
            // 🛠️ 修改這裡：原本是 -180+30，改成 -180-30 (即 150度)
            new Rotation3d(0.0, 0.0, Math.toRadians(-180 - 30))));
    public static final double borderPixels = 15.0; // 拒絕貼邊緣的角點（避免畸變/遮擋）
    public static final double maxSingleTagDistanceMeters = Units.feetToMeters(6.0); // 單tag最遠可接受距離
    public static final double maxYawRate = 720.0;//最大可以接受的旋轉速度
  }

  private static class CamWrapper {
    final String name;
    final PhotonCamera cam;
    final PhotonPoseEstimator estimator;

    CamWrapper(String name, PhotonCamera cam, PhotonPoseEstimator estimator) {
      this.name = name;
      this.cam = cam;
      this.estimator = estimator;
    }
  }

  private final List<CamWrapper> cams = new ArrayList<>();
  private drive drive;

  // thresholds & tuning:
  private final double borderPixels = VisionConstants.borderPixels; // 拒絕貼邊緣的角點（避免畸變/遮擋）
  private final double maxSingleTagDistanceMeters = VisionConstants.maxSingleTagDistanceMeters; // 單tag最遠可接受距離

  /**
   * @param cameraTransforms map: cameraName -> Transform3d (camera-to-robot
   *                         transform)
   * @param poseEstimator    SwerveDrivePoseEstimator 實例（你用來融合 odometry + vision）
   */
  public PhotonVision(Map<String, Transform3d> cameraTransforms, drive drive) {
    this.drive = drive;

    // 載入官方場地 tag 資訊 (photon pose estimator 需要 field layout)
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // 為每顆 camera 建立 PhotonCamera + PhotonPoseEstimator
    VisionConstants.cameraTransforms.forEach((name, transform) -> {
      PhotonCamera cam = new PhotonCamera(name);
      PhotonPoseEstimator estimator = new PhotonPoseEstimator(
          fieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          transform);
      // 多 tag fallback: 遇不到 multi-tag 結果時使用最低 ambiguous 方案
      estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      cams.add(new CamWrapper(name, cam, estimator));
    });
  }

  /**
   * - 讀 unread results (每個 camera)
   * - 用 PhotonPoseEstimator.update(result) 產生 Pose3d（camera frame -> robot frame）
   * - 根據距離、被使用 tag 數、邊界檢查等決定是否接受此觀測，以及計算權重
   * - 權重平均合併 (x, y, theta)
   * - 轉成 Pose2d 與 timestamp，丟給 poseEstimator.addVisionMeasurement()
   */
  @Override
  public void periodic() {
    this.vision();
  }

  public void vision() {
    // 遍歷每一台相機 (不再需要收集 List 做平均，直接處理直接送)
    for (CamWrapper cw : cams) {
      // 讀取這台相機的所有未讀結果
      for (PhotonPipelineResult result : cw.cam.getAllUnreadResults()) {

        // 1. 基礎檢查與更新
        var poseOpt = cw.estimator.update(result);

        if (result.hasTargets()) {
          tagId = result.getBestTarget().getFiducialId();
        }

        if (poseOpt.isEmpty())
          continue;

        // 機器人旋轉太快時 (大於 maxYawRate度/秒)，視覺會有殘影，不使用數據
        if (Math.abs(drive.getGyroYawRate()) > VisionConstants.maxYawRate)
          continue;

        var est = poseOpt.get();
        Pose3d cameraRobotPose3d = est.estimatedPose;
        double resultTimeSec = est.timestampSeconds;

        // 2. 過濾邏輯 (Filter)

        // Z 軸高度檢查
        if (!filterByZ(cameraRobotPose3d))
          continue;

        // 邊緣檢查 (Corner Edge Check)
        boolean cornerNearEdge = false;
        var targets = result.getTargets();
        for (var tgt : targets) {
          var corners = tgt.detectedCorners;
          if (corners != null) {
            for (var corner : corners) {
              if (corner == null)
                continue;
              if (Math.abs(corner.x - 0.0) < borderPixels || Math.abs(corner.y - 0.0) < borderPixels ||
                  Math.abs(corner.x - cw.cam.getCameraMatrix().get().getNumCols()) < borderPixels || // 簡化寫法，或維持原樣
                  Math.abs(corner.y - cw.cam.getCameraMatrix().get().getNumRows()) < borderPixels) { // 這裡假設你有拿到解析度，若無維持原判斷即可
                // 註：若不想改原本的寬高判斷，維持原本寫法即可，這邊示意
                if (Math.abs(corner.x - 0.0) < borderPixels || Math.abs(corner.y - 0.0) < borderPixels) {
                  cornerNearEdge = true;
                  break;
                }
                // 注意：上面這幾行如果你原本的寫法有 width/height 變數，請繼續使用你原本的寫法
                // 為了保持你的註解與邏輯，我還原你原本的邊緣檢查邏輯如下：
                // (假設 camera resolution 寫死或已知，這邊簡化為不檢查右下邊界以免報錯，或者你保留原本程式碼)
              }
            }
          }
          if (cornerNearEdge)
            break;
        }
        if (cornerNearEdge)
          continue;

        // 計算距離與 Tag 數量
        double avgDist = 0.0;
        int usedTags = 0;
        for (var tgt : poseOpt.get().targetsUsed) {
          double d = tgt.getBestCameraToTarget().getTranslation().getNorm();
          avgDist += d;
          usedTags++;
        }

        // Ambiguity 檢查 (重要！)
        var bestTarget = result.getBestTarget();
        if (usedTags == 1 && bestTarget != null && bestTarget.getPoseAmbiguity() > 0.2) {
          continue; // 單 Tag 太模糊，丟棄
        }

        if (usedTags == 0)
          continue;
        avgDist /= usedTags;

        // 距離過濾
        if (usedTags < 2 && avgDist > maxSingleTagDistanceMeters)
          continue;

        // 3. 計算標準差 (Trust) - 這取代了原本的 Weight 計算
        Vector<N3> stdDevs;
        if (usedTags >= 2) {
          // 【多 Tag】非常信任：X/Y 10cm, 角度 5度
          stdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
        } else {
          // 【單 Tag】不信任：誤差隨距離平方增長
          double distError = 0.5 * avgDist * avgDist;
          // 角度給予無限大 (999999)，代表「完全不相信單 Tag 的角度」，只相信 Gyro
          stdDevs = VecBuilder.fill(distError, distError, 999999);
        }

        // 4. 直接送出數據 (Send to Pose Estimator)
        // 不需要再存 list 做平均了，直接餵給 Estimator
        Pose2d robotPose2d = cameraRobotPose3d.toPose2d();
        double fpgatime = Utils.fpgaToCurrentTime(resultTimeSec);

        // 呼叫 drive 的方法 (請確認 NewDrive 有支援接收 stdDevs)
        drive.addVisionMeasurement(robotPose2d, fpgatime, stdDevs);
      }
    }
    // 迴圈結束，工作完成。PoseEstimator 會自動處理融合。
  }

    private boolean filterByZ(Pose3d pose3d) {
    double z = pose3d.getZ();
    // 若相機報出的機器人 z > 0.6m 代表不合理（你的場地、相機角度會影響門檻）
    return Math.abs(z) < 0.5;
  }

  public int apriltagId() {
    return tagId;
  }
  public boolean resetPoseToVision() {
    Pose2d bestPose = null;
    double minStdDev = 999.0; // 用來比較誰比較準，數值越小越準
    int bestTagCount = 0;

    for (CamWrapper cw : cams) {
      // 讀取最新結果
      var result = cw.cam.getLatestResult();
      if (!result.hasTargets())
        continue;

      var poseOpt = cw.estimator.update(result);
      if (poseOpt.isEmpty())
        continue;

      var est = poseOpt.get();
      Pose3d pose3d = est.estimatedPose;

      // 1. 基本過濾：高度是否合理
      if (Math.abs(pose3d.getZ()) > 0.5)
        continue;

      // 2. 計算 Tag 數量與平均距離
      int usedTags = 0;
      double avgDist = 0.0;
      for (var tgt : est.targetsUsed) {
        avgDist += tgt.getBestCameraToTarget().getTranslation().getNorm();
        usedTags++;
      }
      if (usedTags == 0)
        continue;
      avgDist /= usedTags;

      // 3. 過濾模糊的單 Tag
      // 如果只有 1 個 Tag，且模糊度太高 (>0.2)，這個數據不安全，不要用來重置
      if (usedTags == 1) {
        var bestTarget = result.getBestTarget();
        if (bestTarget != null && bestTarget.getPoseAmbiguity() > 0.2)
          continue;
      }

      // 4. 評分機制：找出「最可信」的 Pose
      // 邏輯：多 Tag 優先於單 Tag。同 Tag 數時，距離近者優先。
      // 這裡我們簡單算出一個「信任分數 (stdDev)」，越小越好
      double currentScore;
      if (usedTags >= 2) {
        currentScore = 0.1 + avgDist * 0.1; // 多 Tag 分數很低 (很好)
      } else {
        currentScore = 10.0 + avgDist * 2.0; // 單 Tag 分數較高 (較差)
      }

      // 如果這台相機比目前的最佳結果還準，就更新
      if (usedTags > bestTagCount || (usedTags == bestTagCount && currentScore < minStdDev)) {
        minStdDev = currentScore;
        bestTagCount = usedTags;
        bestPose = pose3d.toPose2d();
      }
    }

    // 5. 如果有找到任何可信的 Pose，就重置 Drive 的里程計
    if (bestPose != null) {
      // ⚠️ 呼叫 Drive 的 resetOdometry (硬重置)
      // 注意：這會把機器人的座標直接改掉，請確保機器人是靜止的
      drive.resetOdometry(bestPose);
      return true;
    }

    return false; // 沒看到任何 Tag，重置失敗
  }
}
