package frc.robot.subsystems.Shooter.Hood;

import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Angle;

public class HoodIONEO implements HoodIO {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;

    public HoodIONEO(int canID) {
            motor = new SparkMax(canID, MotorType.kBrushless);

            encoder = motor.getEncoder();
            pidController = motor.getClosedLoopController();

            motor.configure(HoodConfig.hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
        
    @Override
    public void setAngle(Angle angle) {
        pidController.setSetpoint(angle.in(Radians), ControlType.kPosition);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
