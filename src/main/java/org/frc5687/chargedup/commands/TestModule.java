package org.frc5687.chargedup.commands;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.subsystems.TestyModule;

public class TestModule extends OutliersCommand{
    private TestyModule _module;
    private OI _oi;

    public TestModule (TestyModule module, OI oi){
        _oi = oi;
        _module = module;
        addRequirements(_module);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        _module.testModule(_oi.getDriveX(), _oi.getDriveY());

        //FIXME find a way to implement pneumatic shifting for one module
        // if (_oi.shiftUp()) {
        //     _module.shiftUp();
        // }

        // if (_oi.shiftDown()) {
        //     _module.shiftDown();
        // }
    }
}
