/* Team 5687 (C)2022 */
package org.frc5687.lib.vision;

import java.util.HashMap;
import java.util.Map;

public class TrackedObjectInfo {

    private final GameElement element;
    private final double x;
    private final double y;
    private final double z;

    private final double vx;
    private final double vy;
    private final double vz;

    public TrackedObjectInfo(
            GameElement element, double x, double y, double z, double vx, double vy, double vz) {
        this.element = element;
        this.x = x;
        this.y = y;
        this.z = z;
        this.vx = vx;
        this.vy = vy;
        this.vz = vz;
    }

    public GameElement getElement() {
        return element;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public double getVX() {
        return vx;
    }

    public double getVY() {
        return vy;
    }

    public double getVZ() {
        return vz;
    }

    public enum GameElement {
        UNKNOWN(0),
        GOAL(1),
        RED_BALL(2),
        BLUE_BALL(3);

        private final int _value;
        private static final Map<Object, Object> map = new HashMap<>();

        GameElement(int value) {
            _value = value;
        }

        static {
            for (GameElement gameElement : GameElement.values()) {
                map.put(gameElement._value, gameElement);
            }
        }

        public static GameElement valueOf(int gameElement) {
            return (GameElement) map.get(gameElement);
        }

        public int getValue() {
            return _value;
        }
    }
}
