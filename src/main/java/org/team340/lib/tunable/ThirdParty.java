package org.team340.lib.tunable;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.function.IntConsumer;
import org.team340.lib.tunable.Tunables.TunableHandler;
import org.team340.lib.util.vendors.RevUtil;

/**
 * Registers {@link TunableHandler} implementations for common third-party APIs.
 */
final class ThirdParty {

    static void registerAll() {
        debouncer();
        pidController();
        profiledPIDController();
        sparkFlex();
        sparkMax();
        talonFX();
        talonFXS();
    }

    /** {@link Debouncer} */
    private static void debouncer() {
        DebounceType[] types = DebounceType.values();
        Tunables.registerHandler(Debouncer.class, (table, debouncer) -> {
            table.value("time", debouncer.getDebounceTime(), debouncer::setDebounceTime);
            table.value(
                "type",
                debouncer.getDebounceType().ordinal(),
                (IntConsumer) v -> {
                    if (v < types.length) debouncer.setDebounceType(types[v]);
                }
            );
        });
    }

    /** {@link PIDController} */
    private static void pidController() {
        Tunables.registerHandler(PIDController.class, (table, pid) -> {
            table.value("kP", pid.getP(), pid::setP);
            table.value("kI", pid.getI(), pid::setI);
            table.value("kD", pid.getD(), pid::setD);
            table.value("iZone", pid.getIZone(), pid::setIZone);
        });
    }

    /** {@link ProfiledPIDController} */
    private static void profiledPIDController() {
        Tunables.registerHandler(ProfiledPIDController.class, (table, pid) -> {
            table.value("kP", pid.getP(), pid::setP);
            table.value("kI", pid.getI(), pid::setI);
            table.value("kD", pid.getD(), pid::setD);
            table.value("iZone", pid.getIZone(), pid::setIZone);
            table.value("maxVelocity", pid.getConstraints().maxVelocity, v ->
                pid.setConstraints(new TrapezoidProfile.Constraints(v, pid.getConstraints().maxAcceleration))
            );
            table.value("maxAcceleration", pid.getConstraints().maxAcceleration, v ->
                pid.setConstraints(new TrapezoidProfile.Constraints(pid.getConstraints().maxVelocity, v))
            );
        });
    }

    /** {@link SparkMax} */
    private static void sparkMax() {
        Tunables.registerHandler(SparkMax.class, (table, spark) -> {
            var pidConfig = spark.configAccessor.closedLoop;
            var ffConfig = pidConfig.feedForward;
            var slot = ClosedLoopSlot.kSlot0;

            table.value("kP", pidConfig.getP(slot), v -> {
                var newConfig = new SparkMaxConfig();
                newConfig.closedLoop.p(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("kI", pidConfig.getI(slot), v -> {
                var newConfig = new SparkMaxConfig();
                newConfig.closedLoop.i(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("kD", pidConfig.getD(slot), v -> {
                var newConfig = new SparkMaxConfig();
                newConfig.closedLoop.d(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("kS", ffConfig.getkS(slot), v -> {
                var newConfig = new SparkMaxConfig();
                newConfig.closedLoop.feedForward.kS(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("kV", ffConfig.getkV(slot), v -> {
                var newConfig = new SparkMaxConfig();
                newConfig.closedLoop.feedForward.kV(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("kA", ffConfig.getkA(slot), v -> {
                var newConfig = new SparkMaxConfig();
                newConfig.closedLoop.feedForward.kA(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("kG", ffConfig.getkG(slot), v -> {
                var newConfig = new SparkMaxConfig();
                newConfig.closedLoop.feedForward.kG(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("kCos", ffConfig.getkCos(slot), v -> {
                var newConfig = new SparkMaxConfig();
                newConfig.closedLoop.feedForward.kCos(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("iZone", pidConfig.getIZone(slot), v -> {
                var newConfig = new SparkMaxConfig();
                newConfig.closedLoop.iZone(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("dFilter", pidConfig.getDFilter(slot), v -> {
                var newConfig = new SparkMaxConfig();
                newConfig.closedLoop.dFilter(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("minOutput", pidConfig.getMinOutput(slot), v -> {
                var newConfig = new SparkMaxConfig();
                newConfig.closedLoop.minOutput(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("maxOutput", pidConfig.getMaxOutput(slot), v -> {
                var newConfig = new SparkMaxConfig();
                newConfig.closedLoop.maxOutput(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });

            var motionConfig = spark.configAccessor.closedLoop.maxMotion;
            TunableTable motionTable = table.getNested("motion");

            motionTable.value("velocity", motionConfig.getCruiseVelocity(slot), v -> {
                var newConfig = new SparkMaxConfig();
                newConfig.closedLoop.maxMotion.cruiseVelocity(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            motionTable.value("acceleration", motionConfig.getMaxAcceleration(slot), v -> {
                var newConfig = new SparkMaxConfig();
                newConfig.closedLoop.maxMotion.maxAcceleration(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            motionTable.value("allowedProfileError", motionConfig.getAllowedProfileError(slot), v -> {
                var newConfig = new SparkMaxConfig();
                newConfig.closedLoop.maxMotion.allowedProfileError(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
        });
    }

    /** {@link SparkFlex} */
    private static void sparkFlex() {
        Tunables.registerHandler(SparkFlex.class, (table, spark) -> {
            var pidConfig = spark.configAccessor.closedLoop;
            var ffConfig = pidConfig.feedForward;
            var slot = ClosedLoopSlot.kSlot0;

            table.value("kP", pidConfig.getP(slot), v -> {
                var newConfig = new SparkFlexConfig();
                newConfig.closedLoop.p(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("kI", pidConfig.getI(slot), v -> {
                var newConfig = new SparkFlexConfig();
                newConfig.closedLoop.i(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("kD", pidConfig.getD(slot), v -> {
                var newConfig = new SparkFlexConfig();
                newConfig.closedLoop.d(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("kS", ffConfig.getkS(slot), v -> {
                var newConfig = new SparkFlexConfig();
                newConfig.closedLoop.feedForward.kS(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("kV", ffConfig.getkV(slot), v -> {
                var newConfig = new SparkFlexConfig();
                newConfig.closedLoop.feedForward.kV(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("kA", ffConfig.getkA(slot), v -> {
                var newConfig = new SparkFlexConfig();
                newConfig.closedLoop.feedForward.kA(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("kG", ffConfig.getkG(slot), v -> {
                var newConfig = new SparkFlexConfig();
                newConfig.closedLoop.feedForward.kG(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("kCos", ffConfig.getkCos(slot), v -> {
                var newConfig = new SparkFlexConfig();
                newConfig.closedLoop.feedForward.kCos(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("iZone", pidConfig.getIZone(slot), v -> {
                var newConfig = new SparkFlexConfig();
                newConfig.closedLoop.iZone(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("dFilter", pidConfig.getDFilter(slot), v -> {
                var newConfig = new SparkFlexConfig();
                newConfig.closedLoop.dFilter(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("minOutput", pidConfig.getMinOutput(slot), v -> {
                var newConfig = new SparkFlexConfig();
                newConfig.closedLoop.minOutput(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            table.value("maxOutput", pidConfig.getMaxOutput(slot), v -> {
                var newConfig = new SparkFlexConfig();
                newConfig.closedLoop.maxOutput(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });

            var motionConfig = spark.configAccessor.closedLoop.maxMotion;
            TunableTable motionTable = table.getNested("motion");

            motionTable.value("velocity", motionConfig.getCruiseVelocity(slot), v -> {
                var newConfig = new SparkFlexConfig();
                newConfig.closedLoop.maxMotion.cruiseVelocity(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            motionTable.value("acceleration", motionConfig.getMaxAcceleration(slot), v -> {
                var newConfig = new SparkFlexConfig();
                newConfig.closedLoop.maxMotion.maxAcceleration(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
            motionTable.value("allowedProfileError", motionConfig.getAllowedProfileError(slot), v -> {
                var newConfig = new SparkFlexConfig();
                newConfig.closedLoop.maxMotion.allowedProfileError(v, slot);
                RevUtil.configEphemeral(spark, newConfig);
            });
        });
    }

    /** {@link TalonFX} */
    private static void talonFX() {
        Tunables.registerHandler(TalonFX.class, (table, talonFX) -> {
            Slot0Configs pidConfig = new Slot0Configs();
            talonFX.getConfigurator().refresh(pidConfig);

            table.value("kP", pidConfig.kP, v -> {
                talonFX.getConfigurator().refresh(pidConfig);
                pidConfig.kP = v;
                talonFX.getConfigurator().apply(pidConfig);
            });
            table.value("kI", pidConfig.kI, v -> {
                talonFX.getConfigurator().refresh(pidConfig);
                pidConfig.kI = v;
                talonFX.getConfigurator().apply(pidConfig);
            });
            table.value("kD", pidConfig.kD, v -> {
                talonFX.getConfigurator().refresh(pidConfig);
                pidConfig.kD = v;
                talonFX.getConfigurator().apply(pidConfig);
            });
            table.value("kS", pidConfig.kS, v -> {
                talonFX.getConfigurator().refresh(pidConfig);
                pidConfig.kS = v;
                talonFX.getConfigurator().apply(pidConfig);
            });
            table.value("kV", pidConfig.kV, v -> {
                talonFX.getConfigurator().refresh(pidConfig);
                pidConfig.kV = v;
                talonFX.getConfigurator().apply(pidConfig);
            });
            table.value("kA", pidConfig.kA, v -> {
                talonFX.getConfigurator().refresh(pidConfig);
                pidConfig.kA = v;
                talonFX.getConfigurator().apply(pidConfig);
            });
            table.value("kG", pidConfig.kG, v -> {
                talonFX.getConfigurator().refresh(pidConfig);
                pidConfig.kG = v;
                talonFX.getConfigurator().apply(pidConfig);
            });

            MotionMagicConfigs motionConfig = new MotionMagicConfigs();
            talonFX.getConfigurator().refresh(motionConfig);
            TunableTable motionTable = table.getNested("motion");

            motionTable.value("velocity", motionConfig.MotionMagicCruiseVelocity, v -> {
                talonFX.getConfigurator().refresh(motionConfig);
                motionConfig.MotionMagicCruiseVelocity = v;
                talonFX.getConfigurator().apply(motionConfig);
            });
            motionTable.value("acceleration", motionConfig.MotionMagicAcceleration, v -> {
                talonFX.getConfigurator().refresh(motionConfig);
                motionConfig.MotionMagicAcceleration = v;
                talonFX.getConfigurator().apply(motionConfig);
            });
            motionTable.value("jerk", motionConfig.MotionMagicJerk, v -> {
                talonFX.getConfigurator().refresh(motionConfig);
                motionConfig.MotionMagicJerk = v;
                talonFX.getConfigurator().apply(motionConfig);
            });
            motionTable.value("expoKv", motionConfig.MotionMagicExpo_kV, v -> {
                talonFX.getConfigurator().refresh(motionConfig);
                motionConfig.MotionMagicExpo_kV = v;
                talonFX.getConfigurator().apply(motionConfig);
            });
            motionTable.value("expoKa", motionConfig.MotionMagicExpo_kA, v -> {
                talonFX.getConfigurator().refresh(motionConfig);
                motionConfig.MotionMagicExpo_kA = v;
                talonFX.getConfigurator().apply(motionConfig);
            });
        });
    }

    /** {@link TalonFXS} */
    private static void talonFXS() {
        Tunables.registerHandler(TalonFXS.class, (table, talonFXS) -> {
            Slot0Configs pidConfig = new Slot0Configs();
            talonFXS.getConfigurator().refresh(pidConfig);

            table.value("kP", pidConfig.kP, v -> {
                talonFXS.getConfigurator().refresh(pidConfig);
                pidConfig.kP = v;
                talonFXS.getConfigurator().apply(pidConfig);
            });
            table.value("kI", pidConfig.kI, v -> {
                talonFXS.getConfigurator().refresh(pidConfig);
                pidConfig.kI = v;
                talonFXS.getConfigurator().apply(pidConfig);
            });
            table.value("kD", pidConfig.kD, v -> {
                talonFXS.getConfigurator().refresh(pidConfig);
                pidConfig.kD = v;
                talonFXS.getConfigurator().apply(pidConfig);
            });
            table.value("kS", pidConfig.kS, v -> {
                talonFXS.getConfigurator().refresh(pidConfig);
                pidConfig.kS = v;
                talonFXS.getConfigurator().apply(pidConfig);
            });
            table.value("kV", pidConfig.kV, v -> {
                talonFXS.getConfigurator().refresh(pidConfig);
                pidConfig.kV = v;
                talonFXS.getConfigurator().apply(pidConfig);
            });
            table.value("kA", pidConfig.kA, v -> {
                talonFXS.getConfigurator().refresh(pidConfig);
                pidConfig.kA = v;
                talonFXS.getConfigurator().apply(pidConfig);
            });
            table.value("kG", pidConfig.kG, v -> {
                talonFXS.getConfigurator().refresh(pidConfig);
                pidConfig.kG = v;
                talonFXS.getConfigurator().apply(pidConfig);
            });

            MotionMagicConfigs motionConfig = new MotionMagicConfigs();
            talonFXS.getConfigurator().refresh(motionConfig);
            TunableTable motionTable = table.getNested("motion");

            motionTable.value("velocity", motionConfig.MotionMagicCruiseVelocity, v -> {
                talonFXS.getConfigurator().refresh(motionConfig);
                motionConfig.MotionMagicCruiseVelocity = v;
                talonFXS.getConfigurator().apply(motionConfig);
            });
            motionTable.value("acceleration", motionConfig.MotionMagicAcceleration, v -> {
                talonFXS.getConfigurator().refresh(motionConfig);
                motionConfig.MotionMagicAcceleration = v;
                talonFXS.getConfigurator().apply(motionConfig);
            });
            motionTable.value("jerk", motionConfig.MotionMagicJerk, v -> {
                talonFXS.getConfigurator().refresh(motionConfig);
                motionConfig.MotionMagicJerk = v;
                talonFXS.getConfigurator().apply(motionConfig);
            });
            motionTable.value("expoKv", motionConfig.MotionMagicExpo_kV, v -> {
                talonFXS.getConfigurator().refresh(motionConfig);
                motionConfig.MotionMagicExpo_kV = v;
                talonFXS.getConfigurator().apply(motionConfig);
            });
            motionTable.value("expoKa", motionConfig.MotionMagicExpo_kA, v -> {
                talonFXS.getConfigurator().refresh(motionConfig);
                motionConfig.MotionMagicExpo_kA = v;
                talonFXS.getConfigurator().apply(motionConfig);
            });
        });
    }
}
