package org.frc5687.chargedup.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

public class CustomController {
    private final NetworkTableInstance _instance;
    private final NetworkTable _table;
    private final CustomButton[][] _buttons = new CustomButton[3][9];
    private final CustomButton _override;
    private final CustomButton _changeMode;
    private final CustomButton _deploy;
    private final CustomButton _intake;

    public CustomController() {
        _instance = NetworkTableInstance.getDefault();
        _table = _instance.getTable("dashboard");
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 9; col++) {
                _buttons[row][col] = new CustomButton(row, col, _table);
            }
        }
        _override = new CustomButton("override", _table);
        _changeMode = new CustomButton("changeMode", _table);
        _deploy = new CustomButton("deploy", _table);
        _intake = new CustomButton("intake", _table);
    }

    public void pollButtons() {
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 9; col++) {
                String buttonName = "button_" + row + "_" + col;
                boolean state = _table.getEntry(buttonName).getBoolean(false);
            }
        }
    }

    public Trigger getButton(int row, int col) {
        return new Trigger(_buttons[row][col]);
    }
    public Trigger getOverrideButton() {return new Trigger(_override);}
    public Trigger getChangeModeButton() {return new Trigger(_changeMode);}
    public Trigger getDeployButton() {return new Trigger(_deploy);}
    public Trigger getIntakeButton() {return new Trigger(_intake);}

    public boolean getState(int row, int col) {
        String name = "button_" + row + "_" + col;
        return _table.getEntry(name).getBoolean(false);
    }
}

class CustomButton implements BooleanSupplier {
    private String _name;
    private NetworkTable _table;

    public CustomButton(int row, int col, NetworkTable table) {
        _table = table;
        _name = "button_" + row + "_" + col;
    }

    public CustomButton(String name, NetworkTable table) {
        _table = table;
        _name = name;
    }

    @Override
    public boolean getAsBoolean() {
        return _table.getEntry(_name).getBoolean(false);
    }
}
