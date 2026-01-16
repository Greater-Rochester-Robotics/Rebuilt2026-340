package org.team340.lib.logging.revlib;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import org.team340.lib.logging.revlib.structs.SparkBaseStruct;

@CustomLoggerFor(SparkMax.class)
public class SparkMaxLogger extends ClassSpecificLogger<SparkMax> {

    private static final SparkMaxStruct struct = new SparkMaxStruct();

    public SparkMaxLogger() {
        super(SparkMax.class);
    }

    @Override
    public void update(EpilogueBackend backend, SparkMax sparkMax) {
        backend.log("", sparkMax, struct);
    }

    private static class SparkMaxStruct extends SparkBaseStruct<SparkMax> {

        @Override
        public Class<SparkMax> getTypeClass() {
            return SparkMax.class;
        }

        @Override
        public String getTypeName() {
            return "SparkMax";
        }
    }
}
