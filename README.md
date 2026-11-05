# example-one

Robot code for team 5588: a swerve drive with AprilTag vision, built on WPILib command-based
programming and [AdvantageKit](https://docs.advantagekit.org/) logging.

| Part | What we use | Where in the code |
| --- | --- | --- |
| Drivetrain | Swerve, REV SPARK MAX/Flex + NEO/Vortex (MAXSwerve defaults) | `subsystems/drive` |
| Gyro | CTRE Pigeon 2.0 | `GyroIOPigeon2` |
| Vision | Limelight 4 (MegaTag 1 + MegaTag 2) | `subsystems/vision` |
| Autos | [BLine](https://bline-docs.pages.dev/) path following | `RobotContainer`, `deploy/autos` |
| Controller | WPILib 2026 (roboRIO) today, structured for Systemcore / WPILib 2027 | see below |

## Getting started

```bash
./gradlew build          # compile, format, and run the unit tests
./gradlew simulateJava   # run the robot in simulation (open AdvantageScope to watch it)
./gradlew deploy         # send the code to the robot
```

The build formats the code automatically (Spotless + google-java-format), so do not fight the
formatter. CI on GitHub runs the same build and fails if formatting or tests break.

## How the code is organized

```
src/main/java/frc/robot/
  Robot.java            Starts logging, runs the command scheduler. Keep it small.
  RobotContainer.java   Creates subsystems, picks real/sim/replay hardware, binds buttons, autos.
  Constants.java        Robot-wide settings (run mode, loop period).
  commands/             Commands that use one or more subsystems (driving, characterization).
  subsystems/
    drive/              Swerve: Drive, Module, and the hardware "IO" layers.
    vision/             Vision: filters AprilTag poses and feeds them to the drive.
  util/                 Small helpers.
src/main/deploy/autos/  BLine path files (edit with https://bline-web.pages.dev/).
src/test/java/          Unit tests. Run with ./gradlew test.
```

### The IO pattern (why it is easy to swap hardware)

Every subsystem is split in two:

- **The logic** (`Drive`, `Module`, `Vision`) decides what to do. It never talks to hardware.
- **The IO** (`ModuleIO`, `GyroIO`, `VisionIO`) is an interface for reading sensors and setting
  outputs. Each device gets its own implementation (`ModuleIOSpark`, `ModuleIOSim`,
  `GyroIOPigeon2`, `VisionIOLimelight`, ...).

`RobotContainer` is the only place that chooses which IO to use. This gives us:

1. **Simulation for free.** `ModuleIOSim` stands in for the motors, so autos and driving can be
   tested on a laptop.
2. **Log replay.** AdvantageKit records every IO input. `./gradlew replayWatch` re-runs the code
   against a match log, so a bug from a match can be reproduced and fixed without the robot.
3. **Swappable hardware.** A new device is one new class plus one line in `RobotContainer`.

## Swapping the vision device

`Vision` only sees `VisionIO`, so it does not know it is reading a Limelight. To use something
else (for example PhotonVision):

1. Add the device's vendor library (WPILib: *Manage Vendor Libraries*).
2. Create `VisionIO<Device>.java` in `subsystems/vision` that implements `VisionIO` and fills in
   `VisionIOInputs` (connected, target angles, pose observations with timestamps, tag IDs).
3. In `RobotContainer`, replace `new VisionIOLimelight(...)` with your new class. Add more cameras
   by passing more IO objects to `new Vision(...)` and adding a factor for each one in
   `VisionConstants.kCameraStdDevFactors`.

Filtering and trust settings (`kMaxAmbiguity`, standard deviations, ...) are in `VisionConstants`
and apply to every camera type.

### Limelight 4 notes

- Set the Limelight's name to `limelight` (or change `VisionConstants.kCamera0Name`).
- Set the camera's position on the robot in the Limelight web UI. The robot pose it sends depends
  on it.
- The code sends the robot heading every loop for MegaTag 2, and switches the Limelight 4 IMU mode
  (seed from the Pigeon while disabled, internal IMU while enabled). For an older Limelight, set
  both `kLimelightImuMode*` constants to `0`.

## First time on a real robot (tuning checklist)

Do these in order, with the robot on blocks first.

1. **CAN IDs.** Match `DriveConstants` to the wiring. Check that the Pigeon and all 8 SPARKs
   show up and no "Disconnected" alerts appear.
2. **Module offsets.** Set `k*ZeroRotation` in `DriveConstants` (REV calibration tool for
   MAXSwerve, or read `Drive/Module<N>/TurnPosition` with all wheels pointing forward).
3. **Directions.** Push the stick forward: every wheel should spin to drive forward, and the pose
   in AdvantageScope should move forward. Turning left should make the gyro angle go up.
4. **Wheel radius.** Run the *Drive Wheel Radius Characterization* auto on carpet, update
   `kWheelRadiusMeters`.
5. **Feedforward.** Run *Drive Simple FF Characterization*, update `kDriveKs` / `kDriveKv`.
   Then tune `kDriveKp` and `kTurnKp`.
6. **Max speed.** Measure the real top speed, update `kMaxSpeedMetersPerSec`.
7. **Vision.** Check `Vision/Summary/RobotPosesAccepted` lines up with the odometry pose.
8. **Autos.** Tune the `kPath*Kp` gains in `DriveConstants` with the example path.

## Moving to Systemcore (WPILib 2027)

Systemcore needs WPILib 2027, which is still in alpha. This project stays on WPILib 2026 until the
2027 release and vendor libraries are stable. It is set up so the move is mostly mechanical:

1. **Import the project** with the WPILib 2027 VS Code importer. It updates `build.gradle`, and
   it renames packages from `edu.wpi.first.*` to `org.wpilib.*`. Java 25 is required.
2. **Update every vendordep** to its 2027 version: AdvantageKit, REVLib, Phoenix 6 and BLine
   (BLine has a `wpilib-2027` branch). Also change `java-version` in
   `.github/workflows/build.yml` to 25.
3. **Gyro.** Systemcore has a built-in IMU. Either keep the Pigeon (only `kCanBusName` may need to
   change, because Systemcore has several CAN buses), or add a `GyroIOSystemcore` class that
   implements `GyroIO` and use it in `RobotContainer`. No other drive code changes.
4. **Constant names.** WPILib 2027 uses ALL_CAPS for constants such as `Rotation2d.kZero`
   (becomes `Rotation2d.ZERO`) and renames some methods (for example, speeds become
   velocities). Fix whatever the compiler reports. Also note that `Rotation2d.getRadians()` /
   `getDegrees()` now wrap the angle. The compiler will not catch this, but the wheel radius
   characterization already handles it.
5. **Test mode** is renamed "Utility" mode in 2027, so rename `testInit` / `testPeriodic` in
   `Robot.java`.
6. **Odometry thread.** `SparkOdometryThread` reads `RobotController.getFPGATime()`. Check that it
   still returns microseconds in your 2027 version (some 2027 time functions moved to nanoseconds).

Hardware-specific code is only in the `*IO*` classes, `SparkUtil` and `SparkOdometryThread`,
so the subsystems, commands and autos should need no changes beyond the renames above.

## Tools

- **AdvantageScope**: logs, live data, 3D field view. Comes with the WPILib installer.
- **Elastic**: driver dashboard. Comes with the WPILib installer.
- **BLine web editor**: <https://bline-web.pages.dev/>.
