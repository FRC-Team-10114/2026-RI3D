package frc.robot.subsystems.Hopper.WashingMechine;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.Hopper.HopperConstant.WashingMechineConstants;

public class WashingMechineIOHardware implements WashingMechineIO {

    private final TalonFX mechineMotor;

    private final VoltageOut output;

    public WashingMechineIOHardware() {
        this.mechineMotor = new TalonFX(WashingMechineConstants.MECHINE_MOTOR_ID, "canivore");
        this.output = new VoltageOut(Volts.of(0)); 

        configure();
    }

    @Override
    public void run() {
        this.mechineMotor.setControl(output.withOutput(Volts.of( 10)));
    }

    @Override
    public void stop() {
        this.mechineMotor.stopMotor();
    }

    @Override
    public void configure() {
        var mechineConfig = new TalonFXConfiguration();

        mechineConfig.MotorOutput
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        mechineConfig.CurrentLimits
                .withStatorCurrentLimit(WashingMechineConstants.MECHINE_STATOR_CURRENT_LIMIT)
                .withSupplyCurrentLimit(WashingMechineConstants.MECHINE_SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);
        
        mechineMotor.getConfigurator().apply(mechineConfig);
    }
    
}
