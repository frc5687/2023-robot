/* Team 5687 (C)2021 */
/* Team 5687 (C)2021-2022 */
package org.frc5687.chargedup;

import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.hardware.Pigeon2;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frc5687.chargedup.commands.Arm.ManualDriveArm;
import org.frc5687.chargedup.commands.Auto.*;
import org.frc5687.chargedup.commands.CubeShooter.IdleWrist;
import org.frc5687.chargedup.commands.Drive;
import org.frc5687.chargedup.commands.DriveLights;
import org.frc5687.chargedup.commands.Elevator.ManualExtendElevator;
import org.frc5687.chargedup.commands.EndEffector.IdleGripper;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.*;
import org.frc5687.chargedup.util.*;
import org.frc5687.chargedup.util.AutoChooser.AutoType;
import org.frc5687.chargedup.util.AutoChooser.Node;
import org.frc5687.lib.vision.VisionProcessor;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private AutoChooser _autoChooser;
    private Node _autoFirstNode;
    private AutoType _autoType;
    private VisionProcessor _visionProcessor;
    private Pigeon2 _imu;
    private Robot _robot;
    private Lights _lights;
    // private LightsExample _lights;
    private DriveTrain _driveTrain;
    private EndEffector _endEffector;
    private CubeShooter _cubeShooter;
    private Arm _arm;
    private Elevator _elevator;
    private PhotonProcessor _photonProcessor;
    private Trajectories _trajectories;

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
        _autoType = AutoType.Unknown;
        // create the vision processor
        _visionProcessor = new VisionProcessor();
        // subscribe to a vision topic for the correct data
        _visionProcessor.createSubscriber("vision", "tcp://10.56.87.20:5557");
        _trajectories = new Trajectories(new PathConstraints(3.0, 2.0));

        try {
            _photonProcessor =
//                    new PhotonProcessor(AprilTagFieldLayout.loadFromResource("2023-chargedup.json"));
            new PhotonProcessor(FieldConstants.aprilTags);
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
        _cubeShooter = new CubeShooter(this);
        _endEffector = new EndEffector(this);
        _lights = new Lights(this, _driveTrain, _endEffector, _oi);
        //         This is for auto temporarily, need to fix for both in future.
        _endEffector.setCubeState();

        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _endEffector, _oi));
        setDefaultCommand(_elevator, new ManualExtendElevator(_elevator, _oi));
        setDefaultCommand(_arm, new ManualDriveArm(_arm, _oi));
        setDefaultCommand(_endEffector, new IdleGripper(_endEffector, _oi));
        setDefaultCommand(_lights, new DriveLights(_endEffector, _lights, _driveTrain, _oi));
        setDefaultCommand(_cubeShooter, new IdleWrist(_cubeShooter, _driveTrain, _endEffector));

        _oi.initializeButtons(_driveTrain, _endEffector, _arm, _elevator, _cubeShooter, _lights);

        _visionProcessor.start();
        _robot.addPeriodic(this::controllerPeriodic, 0.005, 0.00);
        _driveTrain.startModules();
        startPeriodic();
        //        _driveTrain.plotTrajectory(TrajectoryGenerator.generateTrajectory(
        //                Constants.Auto.TrajectoryPoints.Node8.RED_NODE_EIGHT_TRAJECTORY_ONE,
        // _driveTrain.getConfig()), "one");
        //        _driveTrain.plotTrajectory(TrajectoryGenerator.generateTrajectory(
        //                Constants.Auto.TrajectoryPoints.Node8.RED_NODE_EIGHT_TRAJECTORY_TWO,
        // _driveTrain.getConfig()), "Two");
        //        _driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_EIGHT_GOAL);
        //        _driveTrain.plotTrajectory(TrajectoryGenerator.generateTrajectory(
        //                Constants.Auto.TrajectoryPoints.Node2.RED_NODE_TWO_TRAJECTORY_ONE,
        // _driveTrain.getConfig()));
    }

    public void periodic() {}

    public void disabledPeriodic() {
        _autoChooser.updateChooser();
        _autoFirstNode = _autoChooser.getFirstNode();
        _autoType = _autoChooser.getAutoType();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void teleopInit() {}

    @Override
    public void autonomousInit() {
        // _autoChooser.updateChooser();
    }

    private void setDefaultCommand(OutliersSubsystem subSystem, OutliersCommand command) {
        if (subSystem == null || command == null) {
            return;
        }
        CommandScheduler s = CommandScheduler.getInstance();
        s.setDefaultCommand(subSystem, command);
    }

    public Command getAutoCommand() {
        // if (_autoFirstNode == Node.OneCone) {
        //     return new OneConeAuto(_driveTrain, _arm, _elevator, _endEffector);
        // } else if (_autoFirstNode == Node.FiveCube) {
        //     return new OneConeLevelAuto(_driveTrain, _arm, _elevator, _endEffector);
        // } else if (_autoFirstNode == Node.NineCone) {
        //     return new OneConeAuto(_driveTrain, _arm, _elevator, _endEffector);
        // } else {
        //     return new WaitCommand(15);
        // }
        error("Current mode is: " + _autoType);
        switch (_autoType) {
            case DriveForTime:
                switch (_autoFirstNode) {
                    case OneCone:
                        return new OneConeAuto(_driveTrain, _arm, _elevator, _endEffector);
                    case TwoCube:
                        return new OneCubeAuto(_driveTrain, _arm, _elevator, _endEffector);
                    case ThreeCone:
                        return new AutoPlaceAndStowHighCube(_elevator, _endEffector, _arm);
                    case FourCone:
                        return new AutoPlaceAndStowHighCube(_elevator, _endEffector, _arm);
                    case FiveCube:
                        return new AutoPlaceAndStowHighCube(_elevator, _endEffector, _arm);
                    case SixCone:
                        return new AutoPlaceAndStowHighCube(_elevator, _endEffector, _arm);
                    case SevenCone:
                        return new AutoPlaceAndStowHighCube(_elevator, _endEffector, _arm);
                    case EightCube:
                        return new OneCubeAuto(_driveTrain, _arm, _elevator, _endEffector);
                    case NineCone:
                        return new OneConeAuto(_driveTrain, _arm, _elevator, _endEffector);
                    default:
                        error("uh oh");
                        return new WaitCommand(15);
                }
            case DrivetoLevel:
                switch (_autoFirstNode) {
                    case OneCone:
                        return new OneConeAuto(_driveTrain, _arm, _elevator, _endEffector);
                    case TwoCube:
                        return new OneCubeAuto(_driveTrain, _arm, _elevator, _endEffector);
                    case ThreeCone:
                        return new AutoPlaceAndStowHighCone(_elevator, _endEffector, _arm);
                    case FourCone:
                        return new OneConeLevelAuto(_driveTrain, _arm, _elevator, _endEffector);
                    case FiveCube:
                        return new OneCubeLevelAuto(_driveTrain, _arm, _elevator, _endEffector);
                    case SixCone:
                        return new OneConeLevelAuto(_driveTrain, _arm, _elevator, _endEffector);
                    case SevenCone:
                        return new AutoPlaceAndStowHighCone(_elevator, _endEffector, _arm);
                    case EightCube:
                        return new OneCubeAuto(_driveTrain, _arm, _elevator, _endEffector);
                    case NineCone:
                        return new OneConeAuto(_driveTrain, _arm, _elevator, _endEffector);
                    default:
                        error("uh oh");
                        return new WaitCommand(15);
                }
            case TwoPiece:
                return new TwoPieceAuto(
                    _driveTrain,
                    _endEffector,
                    _elevator,
                    _arm,
                    _lights,
                    _cubeShooter,
                    _oi,
                    _autoFirstNode,
                    _trajectories);
            case TwoAndAHalfPieceAuto:
                return new TwoAndAHalfPieceAuto(
                    _driveTrain, 
                    _endEffector, 
                    _elevator, 
                    _arm, 
                    _lights, 
                    _cubeShooter, 
                    _oi, 
                    _autoFirstNode, 
                    _trajectories);
            case TwoAndAHalfPieceLevelAuto:
                return new TwoAndAHalfPieceLevelAuto(
                    _driveTrain, 
                    _endEffector, 
                    _elevator, 
                    _arm, 
                    _lights,
                    _cubeShooter, 
                    _oi, 
                    _autoFirstNode, 
                    _trajectories);
            case StealCubes:
                return new StealCubesAuto(
                        _driveTrain, _endEffector, _elevator, _arm, _lights, _cubeShooter, _oi, _trajectories);
            case ThreeCubeLevel:
                return new ThreePieceLevel(
                        _driveTrain, _arm, _elevator, _endEffector, _lights, _oi, _cubeShooter, _trajectories);
            case ThreeCubeNoLevel:
                return new ThreePieceNoLevel(_driveTrain, _arm, _elevator, _endEffector, _lights, _oi, _cubeShooter, _trajectories);
            case OneAndAHalfLevel:
                return new OneAndAHalfLevel(_driveTrain, _arm, _elevator, _endEffector, _cubeShooter);
            case ThreePiece:
                return new ThreePieceAuto(
                    _driveTrain, 
                    _endEffector, 
                    _elevator, 
                    _arm, 
                    _lights, 
                    _cubeShooter, 
                    _oi, 
                    _autoFirstNode, 
                    _trajectories); 
            default:
                error("big uh oh");
                return new WaitCommand(15);
        }
    }

    public void controllerPeriodic() {
        if (_driveTrain != null) {
            _driveTrain.modulePeriodic();
        }
    }
}
