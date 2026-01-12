package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    
    private final IntakeIO io;

    private double velocity;

    public IntakeSubsystem(IntakeIO io) {

        this.io = io;
        this.velocity = 0.0;
    }

    @Override
    public void periodic() {
        this.io.setControl(velocity, false);
    }

    public void intake() {
        this.velocity = 0.5;
    }

    public void outtake() {
        this.velocity = -0.5;
    }

    public void stopMotor() {
        this.velocity = 0.0;
    }

}
