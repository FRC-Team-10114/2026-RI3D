package frc.robot.subsystems.Intake;

public interface IntakeIO {

    public void setControl(double velocity, boolean isFOC);

    public double getVelocity();
    
    public void configure();
}
