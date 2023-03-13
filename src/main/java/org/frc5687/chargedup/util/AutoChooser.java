package org.frc5687.chargedup.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoChooser extends OutliersProxy {

    private SendableChooser<Node> _firstNodeChooser;
    private Node _firstNode;

    public AutoChooser() {
        _firstNodeChooser = new SendableChooser<>();
        _firstNodeChooser.setDefaultOption("One", Node.OneCone);
        _firstNodeChooser.addOption("Two", Node.TwoCube);
        _firstNodeChooser.addOption("Three", Node.ThreeCone);
        _firstNodeChooser.addOption("Four", Node.FourCone);
        _firstNodeChooser.addOption("Five", Node.FiveCube);
        _firstNodeChooser.addOption("Six", Node.SixCone);
        _firstNodeChooser.addOption("Seven", Node.SevenCone);
        _firstNodeChooser.addOption("Eight", Node.EightCube);
        _firstNodeChooser.addOption("Nine", Node.NineCone);
        SmartDashboard.putData("First Node", _firstNodeChooser);
    }

    public void updateChooser() {
        _firstNode = _firstNodeChooser.getSelected();
        
        metric("First Piece", _firstNode.name());
    }

    public Node getFirstNode() {
        return _firstNode;
    }

    @Override
    public void updateDashboard() {}

    // public enum Piece {
    //     Unknown(-1),
    //     Cone(0),
    //     Cube(1);

    //     private int _value;

    //     Piece(int value) {
    //         _value = value;
    //     }

    //     public int getValue() {
    //         return _value;
    //     }
    // }

    public enum Node {
        Unknown(-1),
        OneCone(0),
        TwoCube(1),
        ThreeCone(2),
        FourCone(3),
        FiveCube(4),
        SixCone(5),
        SevenCone(6),
        EightCube(7),
        NineCone(8);

        private int _value;

        Node(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
