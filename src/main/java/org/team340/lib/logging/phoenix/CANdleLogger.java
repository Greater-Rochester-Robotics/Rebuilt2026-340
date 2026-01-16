package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(CANdle.class)
public class CANdleLogger extends ClassSpecificLogger<CANdle> {

    public CANdleLogger() {
        super(CANdle.class);
    }

    @Override
    public void update(EpilogueBackend backend, CANdle candle) {
        // No-op
    }
}
