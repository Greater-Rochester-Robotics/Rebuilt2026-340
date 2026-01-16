package org.team340.lib.logging.revlib;

import com.revrobotics.spark.SparkLimitSwitch;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(SparkLimitSwitch.class)
public class SparkLimitSwitchLogger extends ClassSpecificLogger<SparkLimitSwitch> {

    public SparkLimitSwitchLogger() {
        super(SparkLimitSwitch.class);
    }

    @Override
    public void update(EpilogueBackend backend, SparkLimitSwitch limitSwitch) {
        backend.log("isPressed", limitSwitch.isPressed());
    }
}
