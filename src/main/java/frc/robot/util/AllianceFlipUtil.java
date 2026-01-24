package frc.robot.util; // 記得改成你的 package 名稱

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants; // 記得引入你有 fieldLength 的那個檔案

/**
 * 用於處理紅藍方座標翻轉的工具類別
 */
public class AllianceFlipUtil {
    
    /**
     * 如果是紅方，將 Translation2d 進行鏡像翻轉
     */
    public static Translation2d apply(Translation2d translation) {
        if (shouldFlip()) {
            return new Translation2d(
                Constants.fieldLength - translation.getX(), 
                translation.getY()
            );
        }
        return translation;
    }

    /**
     * 如果是紅方，將 Pose2d 進行鏡像翻轉
     */
    public static Pose2d apply(Pose2d pose) {
        if (shouldFlip()) {
            return new Pose2d(
                Constants.fieldLength - pose.getX(),
                pose.getY(),
                new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()) // 角度也要翻轉 (180 - theta)
            );
        }
        return pose;
    }

    /**
     * 判斷是否需要翻轉 (當我們是紅隊時需要翻轉)
     */
    public static boolean shouldFlip() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == Alliance.Red;
        }
        return false; // 預設不翻轉 (當作藍隊)
    }
}