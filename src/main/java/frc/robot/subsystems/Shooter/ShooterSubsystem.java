package frc.robot.subsystems.Shooter;

import java.util.function.Supplier;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.Hood.HoodIO;
import frc.robot.subsystems.Shooter.Roller.RollerIO;
import frc.robot.subsystems.Shooter.Turret.TurretIO;

public class ShooterSubsystem extends SubsystemBase {
    
    private final HoodIO hood;
    private final RollerIO roller;
    private final TurretIO turret;
    private final ShooterCalculator shooterCalculator;

    public ShooterSubsystem(HoodIO hood, RollerIO roller, TurretIO turret,ShooterCalculator shooterCalculator) {

        this.hood = hood;
        this.roller = roller;
        this.turret = turret;
        this.shooterCalculator = shooterCalculator;
        
        this.turret.resetAngle();
    }


    @Override
    public void periodic() {
        this.shooterCalculator.predictedcoordinates();
    }

    public void setHoodAngle(Angle targetRad) {
        this.hood.setAngle(targetRad); 
    }

    public void setTurretAngle( Angle targetRad) {
        this.turret.setControl(targetRad);
    }

    public void setRollerVelocity(AngularVelocity velocity) {
        this.roller.setRPM(velocity);
    }
}
