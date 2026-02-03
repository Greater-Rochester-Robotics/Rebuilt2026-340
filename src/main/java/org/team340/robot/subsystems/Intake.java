package org.team340.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants.RobotMap;

public class Intake extends GRRSubsystem {

    private TalonFX pivot;
    private TalonFX rollers;
    private CANcoder wcpThroughborePoweredByCANcoderForHalfInchHex;

    public Intake() {
        this.pivot = new TalonFX(RobotMap.INTAKE_PIVOT_MOTOR, RobotMap.CANBus);
        this.rollers = new TalonFX(RobotMap.INTAKE_ROLLER_MOTOR, RobotMap.CANBus);
        this.wcpThroughborePoweredByCANcoderForHalfInchHex = new CANcoder(
            RobotMap.INTAKE_WCP_THROUGHBORE_POWERED_BY_CANCODER_FOR_HALF_INCH_HEX,
            RobotMap.CANBus
        );
    }
}
