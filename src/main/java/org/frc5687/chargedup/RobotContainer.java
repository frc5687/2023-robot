/* Team 5687 (C)2021 */
package org.frc5687.chargedup;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import org.frc5687.chargedup.commands.Drive;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.OutliersSubsystem;
import org.frc5687.chargedup.util.OutliersContainer;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.chargedup.util.VisionProcessor;

import java.io.IOException;
import java.util.concurrent.ExecutionException;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private AHRS _imu;
    private Robot _robot;
    private VisionProcessor _visionProcessor;
    private DriveTrain _driveTrain;

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200);
        try {
            _visionProcessor = new VisionProcessor("camera", new AprilTagFieldLayout("deploy/2023-chargedup.json"));
        } catch(IOException e) {
            error("AprilTag map not found");
        }

        _driveTrain = new DriveTrain(this, _oi, _imu, _visionProcessor);

        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));
        _robot.addPeriodic(this::controllerPeriodic, 0.005, 0.005);
        _imu.reset();
    }

    public void periodic() {}

    public void disabledPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void teleopInit() {}

    @Override
    public void autonomousInit() {}

    private void setDefaultCommand(OutliersSubsystem subSystem, OutliersCommand command) {
        if (subSystem == null || command == null) {
            return;
        }
        CommandScheduler s = CommandScheduler.getInstance();
        s.setDefaultCommand(subSystem, command);
    }

    @Override
    public void updateDashboard() {
        _driveTrain.updateDashboard();
    }

    public void controllerPeriodic() {
        if (_driveTrain != null) {
            _driveTrain.controllerPeriodic();
        }
    }
}
