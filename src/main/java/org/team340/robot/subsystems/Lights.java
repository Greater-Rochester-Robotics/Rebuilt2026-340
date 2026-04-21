package org.team340.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.team340.lib.math.Math2;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.tunable.Tunables.TunableInteger;
import org.team340.lib.util.Alliance;
import org.team340.lib.util.Mutable;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants.RobotMap;

@Logged
public final class Lights extends GRRSubsystem {

    private static final int LENGTH = 28;
    private static final TunableTable tunables = Tunables.getNested("lights");

    private static final TunableDouble locationTol = tunables.value("locationTol", 0.05);
    private static final TunableDouble angularTol = tunables.value("angularTol", Math.toRadians(0.5));
    private static final Debouncer tagFilter = tunables.add("tagFilter", new Debouncer(0.2, DebounceType.kFalling));

    private static enum Color {
        HUB_INACTIVE(255, 90, 0),
        HUB_ACTIVE(0, 255, 0),
        BLUE(0, 10, 255),
        RED(255, 0, 0),
        NO_DS(255, 0, 255),
        NO_TAGS(255, 0, 0),
        BAD_LOCATION(255, 144, 0),
        BAD_ROTATION(255, 32, 0),
        OFF(0, 0, 0);

        public final TunableInteger r;
        public final TunableInteger g;
        public final TunableInteger b;

        private Color(int r, int g, int b) {
            this.r = tunables.value("colors/" + name() + "/r", r);
            this.g = tunables.value("colors/" + name() + "/g", g);
            this.b = tunables.value("colors/" + name() + "/b", b);
        }
    }

    private final AddressableLED lights;
    private final AddressableLEDBuffer buffer;

    public Lights() {
        lights = new AddressableLED(RobotMap.LIGHTS);
        buffer = new AddressableLEDBuffer(LENGTH);

        lights.setLength(buffer.getLength());
        lights.start();

        // Enum warmup
        Color.OFF.r.get();
    }

    /**
     * Sets the LED output data.
     * Must be ran periodically for this subsystem to work.
     */
    public void update() {
        lights.setData(buffer);
    }

    /**
     * Displays information for the driver.
     * @param hubActive If the hub is active.
     * @param shooting If the robot is shooting.
     */
    public Command driving(BooleanSupplier hubActive, BooleanSupplier shooting) {
        return hubDisplay(hubActive)
            .until(shooting)
            .andThen(flames(true).onlyWhile(shooting))
            .repeatedly()
            .withName("Lights.driving()");
    }

    /**
     * Displays the state of our alliance's hub.
     * @param active If the hub is active.
     */
    public Command hubDisplay(BooleanSupplier active) {
        Mutable<Boolean> lastRSL = new Mutable<>(false);
        Timer timer = new Timer();

        return commandBuilder()
            .onExecute(() -> {
                boolean rsl = RobotController.getRSLState();
                boolean isActive = active.getAsBoolean();

                if (lastRSL.value != (lastRSL.value = rsl)) {
                    if (isActive || rsl) timer.restart();
                    else timer.stop();
                }

                if (timer.isRunning()) {
                    Color color = isActive ? Color.HUB_ACTIVE : Color.HUB_INACTIVE;
                    double percent = Math.max(1.0 - (timer.get() / (isActive ? 0.1 : 0.2)), 0.0);
                    set(
                        (int) (color.r.get() * percent),
                        (int) (color.g.get() * percent),
                        (int) (color.b.get() * percent)
                    );
                } else {
                    set(Color.OFF);
                }
            })
            .onEnd(() -> set(Color.OFF))
            .ignoringDisable(true)
            .withName("Lights.hubDisplay()");
    }

    /**
     * Displays the flames animation.
     * @param allianceColors If the color of the flames should match the robot's alliance.
     */
    public Command flames(boolean allianceColors) {
        Mutable<Integer> count = new Mutable<>(0);
        int[] state = new int[LENGTH / 2];

        return commandBuilder()
            .onInitialize(() -> {
                count.value = 0;
                for (int i = 0; i < state.length; i++) {
                    state[i] = 0;
                }
            })
            .onExecute(() -> {
                for (int i = 0; i < LENGTH / 2; i++) {
                    state[i] = Math.max(0, state[i] - 28);
                }

                for (int i = LENGTH / 2 - 1; i >= 2; i--) {
                    state[i] = (state[i - 1] + state[i - 2] * 2) / 3;
                }

                if (count.value++ % 2 == 0) {
                    int i = (int) Math2.random(3.0);
                    state[i] = (int) Math.min(255.0, state[i] + Math2.random(200.0, 255.0));
                }

                for (int i = 0; i < LENGTH / 2; i++) {
                    int heat = Math.min(255, state[i] + (Math.random() < 0.1 && state[i] > 16 ? 32 : 0));
                    int ramp = (((int) Math.round((heat / 255.0) * 191.0)) & 63) << 2;

                    if (heat > 170) {
                        if (allianceColors) {
                            if (Alliance.isBlue()) setMirrored(i, ramp, ramp, 255);
                            else setMirrored(i, 255, ramp, ramp);
                        } else {
                            setMirrored(i, 255, 255, ramp / 2);
                        }
                    } else if (heat > 84) {
                        if (allianceColors) {
                            if (Alliance.isBlue()) setMirrored(i, ramp / 4, 0, 255);
                            else setMirrored(i, 255, 0, ramp / 4);
                        } else {
                            setMirrored(i, 255, ramp / 2, 0);
                        }
                    } else {
                        if (allianceColors && Alliance.isBlue()) {
                            setMirrored(i, 0, 0, ramp);
                        } else {
                            setMirrored(i, ramp, 0, 0);
                        }
                    }
                }
            })
            .onEnd(() -> set(Color.OFF))
            .ignoringDisable(true)
            .withName("Lights.flames(" + allianceColors + ")");
    }

    /**
     * Displays the pre-match animation.
     * @param robotPose The robot's current pose.
     * @param seesAprilTag If the robot has seen an AprilTag since the last loop.
     */
    public Command preMatch(Supplier<Pose2d> robotPose, BooleanSupplier seesAprilTag) {
        Debouncer tag = new Debouncer(0.5, DebounceType.kFalling);
        Debouncer location = new Debouncer(0.5, DebounceType.kFalling);
        Debouncer angle = new Debouncer(0.5, DebounceType.kFalling);

        SlewRateLimiter center = new SlewRateLimiter(0.0175);
        Mutable<Pose2d> lastPose = new Mutable<>(Pose2d.kZero);
        Mutable<Double> errorTime = new Mutable<>(-1.0);
        List<Color> errors = new ArrayList<>();

        return commandBuilder()
            .onInitialize(() -> {
                Pose2d pose = robotPose.get();
                center.reset(pose.getRotation().getRadians());
                lastPose.value = pose;
                errorTime.value = -1.0;
            })
            .onExecute(() -> {
                errors.clear();
                Pose2d pose = robotPose.get();

                if (tag.calculate(!tagFilter.calculate(seesAprilTag.getAsBoolean()))) {
                    errors.add(Color.NO_TAGS);
                } else if (
                    location.calculate(
                        !Math2.isNear(lastPose.value.getTranslation(), pose.getTranslation(), locationTol.get())
                    )
                ) {
                    errors.add(Color.BAD_LOCATION);
                } else if (
                    angle.calculate(!Math2.isNear(lastPose.value.getRotation(), pose.getRotation(), angularTol.get()))
                ) {
                    errors.add(Color.BAD_ROTATION);
                }

                lastPose.value = pose;
                if (!errors.isEmpty()) {
                    double now = Timer.getFPGATimestamp();
                    if (errorTime.value < 0.0) errorTime.value = now;

                    double period = (now - errorTime.value) % 0.2;
                    if (period < 0.15) {
                        int i = (int) Math.floor((period / 0.15) * errors.size());
                        set(errors.get(Math.min(i, errors.size())));
                    } else {
                        set(Color.OFF);
                    }
                } else {
                    double radians = pose.getRotation().getRadians();

                    if (errorTime.value > 0.0) {
                        errorTime.value = -1.0;
                        center.reset(radians);
                    }

                    double view = Math.toRadians(1.0);
                    double percent = (center.calculate(radians) + (view / 2.0) - radians) / view;
                    int closestLED = (int) Math.round(percent * (LENGTH - 1));

                    Color alliance = !DriverStation.isDSAttached()
                        ? Color.NO_DS
                        : (Alliance.isBlue() ? Color.BLUE : Color.RED);
                    for (int i = 0; i < LENGTH; i++) {
                        if (Math.abs(closestLED - i) <= 1) {
                            set(i, alliance);
                        } else {
                            set(i, Color.OFF);
                        }
                    }
                }
            })
            .onEnd(() -> set(Color.OFF))
            .ignoringDisable(true)
            .withName("Lights.preMatch()");
    }

    /**
     * Sets the entire LED strip to a single color.
     * @param color The color to apply.
     */
    private void set(Color color) {
        for (int i = 0; i < LENGTH; i++) set(i, color);
    }

    /**
     * Sets the entire LED strip to a single color.
     * @param r Red value from {@code 0} to {@code 255}.
     * @param g Green value from {@code 0} to {@code 255}.
     * @param b Blue value from {@code 0} to {@code 255}.
     */
    private void set(int r, int g, int b) {
        for (int i = 0; i < LENGTH; i++) set(i, r, g, b);
    }

    /**
     * Sets a single LED to a specified color.
     * @param i The index of the LED on the strip.
     * @param color The color to apply.
     */
    private void set(int i, Color color) {
        if (i < 0 || i >= LENGTH) return;
        set(i, color.r.get(), color.g.get(), color.b.get());
    }

    /**
     * Sets a single LED to a specified color.
     * @param i The index of the LED on the strip.
     * @param r Red value from {@code 0} to {@code 255}.
     * @param g Green value from {@code 0} to {@code 255}.
     * @param b Blue value from {@code 0} to {@code 255}.
     */
    private void set(int i, int r, int g, int b) {
        if (i < 0 || i >= LENGTH) return;
        buffer.setRGB(i, r, g, b);
    }

    /**
     * Sets 2 LEDs mirrored across the center of the LED strip to the specified color.
     * @param i The index of the LED on the strip, starting from the center.
     * @param r Red value from {@code 0} to {@code 255}.
     * @param g Green value from {@code 0} to {@code 255}.
     * @param b Blue value from {@code 0} to {@code 255}.
     */
    private void setMirrored(int i, int r, int g, int b) {
        set(LENGTH / 2 - i - 1, r, g, b);
        set(LENGTH / 2 + i, r, g, b);
    }
}
