package frc.robot.subsystems.Intake;

import edu.wpi.first.units.measure.Angle;

public interface ArmIO {

    public void setPosition(Angle position);

    public Angle getPosition();

    public void resetEncoder();

    public void configure();

    
}
