package org.frc5687.chargedup.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

public class CustomController {
    private NetworkTableInstance _instance;
    private NetworkTable _table;
    private CustomButton[][] _buttons = new CustomButton[3][9];

    public CustomController() {
        _instance = NetworkTableInstance.getDefault();
        _table = _instance.getTable("dashboard");
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 9; col++) {
                String buttonName = "button_" + row + "_" + col;
                _buttons[row][col] = new CustomButton(row, col, _table);
            }
        }
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

    @Override
    public boolean getAsBoolean() {
        return _table.getEntry(_name).getBoolean(false);
    }
}
