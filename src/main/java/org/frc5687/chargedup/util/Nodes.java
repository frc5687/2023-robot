package org.frc5687.chargedup.util;

public class Nodes {
    public enum SuperStructurePosition{
        LOW(0),
        MIDDLE(1),
        HIGH(2);

        private final int _value;

        SuperStructurePosition(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }

    public enum Node {
        ONE(0),
        TWO(1),
        THREE(2),
        FOUR(3),
        FIVE(4),
        SIX(5),
        SEVEN(6),
        EIGHT(7),
        NINE(8);

        private final int _value;

        Node(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
