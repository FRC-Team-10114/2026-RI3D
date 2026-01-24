package frc.robot.subsystems.Shooter.Hood;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;

public interface HoodIO {

    public default void setAngle(Angle angle) {}
    public default void setVoltage(double volts) {}
    public default void stop() {}
}
