package frc.robot.subsystems.Hopper;

import frc.robot.subsystems.Hopper.Trigger.TriggerIO;
import frc.robot.subsystems.Hopper.WashingMechine.WashingMechineIO;

public class HopperSubsystem {
    
    private final TriggerIO trigger;
    private final WashingMechineIO washingMechine;

    public HopperSubsystem(
        TriggerIO trigger,
        WashingMechineIO washingMechine
    ) {
        this.trigger = trigger;
        this.washingMechine = washingMechine;
    }

    public void warmUp() {
        this.washingMechine.run();
    }

    public void load() {
        this.trigger.run();
    }

    public void triggerStop() {
        this.trigger.stop();
    }

    public void stopWashing() {
        this.washingMechine.stop();
    }
}
