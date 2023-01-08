# Repository containing the base of 5687 Swerve Drive.

Includes / Will Include :
  - [x] Differential Module State Space Model
  - [x] LQR Control of Modules
  - [ ] Localization Pose Estimator
  - [ ] Trajectory Generation / Following
  - [ ] Maverick
  - [ ] Venom

## Localization:
Will use the WPILib [SwerveDrivePoseEstimator](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose_state-estimators.html) which implements a Uncented Kalman Filter with latency compensation. <br />
#### Devices:
  - Photon Vision with Hardware (Limelight or GloWorm)
    - Pro: Easy drop in, supported by WPILib. 
    - Pro: Can be relativly cheap.
    - Con: Needs retro reflective tape for estimation thus needing to be seeing a target. 
  - ZED 2:
    - Pro: Has SLAM algorithm for locatilation with depth sensing for more ways of localization
    - Pro: Lots of potential if utilized correctly.
    - Pro: Has external GPU for off board calculations.
    - Con: Expensive bundle (ZED 2 ~$400 along with Nvidia Series ~$100-$400)
    - Con: Not as easy to drop in.

## Trajectory Generation:
#### Methods:
  - WPILib Trajectory Generation:
    - Pro: Easy to implement and supported regulary.
    - Con: Doesn't have great heading control for swerve (possible to tweak)
  - Custom:
    - Pro: Can design to your liking.
    - Con: Not simple.
