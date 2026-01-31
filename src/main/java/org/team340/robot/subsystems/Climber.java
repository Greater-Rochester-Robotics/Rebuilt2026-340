package org.team340.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import org.team340.robot.Constants.RobotMap;

public class Climber {

    private final TalonFX lead;
    private final TalonFX follow;

    public Climber() {
        lead = new TalonFX(RobotMap.CLIMBER_LEAD_MOTOR, RobotMap.CANBus);
        follow = new TalonFX(RobotMap.CLIMBER_FOLLOW_MOTOR, RobotMap.CANBus);

        // TODO: Add motor configs
        // TODO: Make follower follow leader (look at Shooter.java)
    }
}
