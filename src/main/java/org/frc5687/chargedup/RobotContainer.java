/* Team 5687 (C)2021 */
/* Team 5687 (C)2021-2022 */
package org.frc5687.chargedup;

import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.hardware.Pigeon2;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.commands.Arm.ManualDriveArm;
import org.frc5687.chargedup.commands.Auto.AutoPlaceHighCone;
import org.frc5687.chargedup.commands.Auto.DriveForTime;
import org.frc5687.chargedup.commands.Auto.OnePieceAuto;
import org.frc5687.chargedup.commands.Drive;
import org.frc5687.chargedup.commands.DriveLights;
import org.frc5687.chargedup.commands.Elevator.ManualExtendElevator;
import org.frc5687.chargedup.commands.EndEffector.IdleGripper;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.*;
import org.frc5687.chargedup.util.AutoChooser;
import org.frc5687.chargedup.util.OutliersContainer;
import org.frc5687.chargedup.util.PhotonProcessor;
import org.frc5687.chargedup.util.AutoChooser.Node;
import org.frc5687.lib.vision.VisionProcessor;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private AutoChooser _autoChooser;
    private Node _autoFirstNode;
    private VisionProcessor _visionProcessor;
    private Pigeon2 _imu;
    private Robot _robot;
    private Lights _lights;
    // private LightsExample _lights;
    private DriveTrain _driveTrain;
    private EndEffector _endEffector;
    private Arm _arm;
    private Elevator _elevator;
    private PhotonProcessor _photonProcessor;

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
        Thread.currentThread().setName("Robot Thread");
        _oi = new OI();
        _autoChooser = new AutoChooser();
        _autoFirstNode = Node.Unknown;
        // create the vision processor
        _visionProcessor = new VisionProcessor();
        // subscribe to a vision topic for the correct data
        _visionProcessor.createSubscriber("vision", "tcp://10.56.87.20:5557");

        try {
            _photonProcessor =
                    new PhotonProcessor(AprilTagFieldLayout.loadFromResource("2023-chargedup.json"));
        } catch (Exception e) {
            e.getMessage();
        }
        // configure pigeon
        _imu = new Pigeon2(RobotMap.CAN.PIGEON.PIGEON, "CANivore");
        var pigeonConfig = new Pigeon2Configuration();
        _imu.getConfigurator().apply(pigeonConfig);

        _driveTrain = new DriveTrain(this, _visionProcessor, _photonProcessor, _imu);
        _elevator = new Elevator(this);
        _arm = new Arm(this);
        _endEffector = new EndEffector(this);
        _lights = new Lights(this, _driveTrain, _endEffector, _oi);
        //         This is for auto temporarily, need to fix for both in future.
        _endEffector.setCubeMode();

        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _endEffector, _oi));
        setDefaultCommand(_elevator, new ManualExtendElevator(_elevator, _oi));
        setDefaultCommand(_arm, new ManualDriveArm(_arm, _oi));
        setDefaultCommand(_endEffector, new IdleGripper(_endEffector, _oi));
        setDefaultCommand(_lights, new DriveLights(_endEffector, _lights, _driveTrain, _oi));
        //        setDefaultCommand(_endEffector, new ManualDriveWrist(_endEffector, _oi));

        _oi.initializeButtons(_driveTrain, _endEffector, _arm, _elevator);

        _visionProcessor.start();
        _robot.addPeriodic(this::controllerPeriodic, 0.005, 0.000);
        _driveTrain.startModules();
        //        startPeriodic();
    }

    public void periodic() {}

    public void disabledPeriodic() {
        _autoChooser.updateChooser();
        _autoFirstNode = _autoChooser.getFirstNode();
    }

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

    public Command getAutoCommand() {
        return new OnePieceAuto(_driveTrain, _arm, _elevator, _endEffector);
    }

    public void controllerPeriodic() {
        if (_driveTrain != null) {
            _driveTrain.modulePeriodic();
        }
    }
}
