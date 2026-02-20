# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Deploy

This is an FTC (FIRST Tech Challenge) Android project using Gradle 8.9 and Android Gradle Plugin 8.7.0. The robot code deploys to an Android-based Robot Controller (REV Control Hub).

```bash
# Build the project
./gradlew build

# Build and install to connected Robot Controller via ADB
./gradlew TeamCode:installDebug

# Clean build
./gradlew clean build
```

The project targets Android SDK 34 (min SDK 24) with Java 8. There is no test suite — verification happens on-robot.

## Project Structure

Two Gradle modules defined in `settings.gradle`:
- **FtcRobotController/** — FTC SDK v11.0 library module (do not modify; contains official samples in `external/samples/`)
- **TeamCode/** — All team-written code lives here

All team source files are in one package: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/`

## Architecture

### Hardware Layer
- **FTChardware.java** — Hardware abstraction. All motors, servos, and odometry are initialized here with names matching the Robot Controller's hardware map configuration. Includes voltage compensation for gecko launch motors.
- **Constants.java** — Pedro Pathing PIDF tuning values, mecanum drive config, odometry offsets, and path constraints. Contains a `createFollower()` factory method.

### OpModes
- **FTCTeleOp2026.java** (`@TeleOp`) — Driver control with mecanum drive, revolver arm, gecko launchers, intake toggle, and ball pusher servo.
- **FTCAutonomous2026.java** (`@Autonomous`) — Main autonomous with tested paths. Uses state machine pattern.
- **FTCAutonomousGPP/PGP/PPG.java** — Sequence-specific autonomous variants for the three possible pile arrangements. Each has different `SPIT_ROT` rotation tables. Currently in development (see `AUTONOMOUS_TODO.md`).

### Test/Tuning OpModes
- **TestIntakeSequence.java** — Tests intake motor with revolver rotation
- **TestSpitOutBall.java** — Tests gecko wheels and ball pusher servo
- **Tuning.java** — Comprehensive Pedro Pathing tuning suite (localization, velocity, PIDF tuners)

### Key Patterns
- **State machine**: Autonomous OpModes use an enum + single state variable with transitions in `loop()`.
- **Pedro Pathing**: Autonomous paths use `BezierLine`/`BezierCurve` with heading interpolation, composed into `PathChain` objects. The `Follower` handles trajectory execution.
- **Revolver motor**: 448.1 ticks/rev (REV HD Hex 20:1). Rotates in 120° increments between 3 ball slots, with 30° final alignment.
- **Voltage compensation**: Gecko wheel powers are scaled by `(12.0 / currentVoltage)` for consistent launch speeds.

### Hardware Map Names
Motors: `frontLeft`, `frontRight`, `backLeft`, `backRight`, `revolverMotor`, `lGecko`, `rGecko`
Servos: `ballPusher` | CR Servo: `rIntake` | Odometry: `odo` (GoBilda Pinpoint)

## DECODE 2025-2026 Field Layout

Reference image: `download.png` (in repo root). Always consult when working on autonomous paths.

```
        BLUE triangle              RED triangle
        (top-left)                 (top-right)
  +------------------------------------------+
  |                                          |
  |   [G P P]  (upper left)  [P P G]  (upper right)
  |                                          |
  ===  BLUE lever                RED lever  ===
  |                                          |
  |   [P G P]  (middle left) [P G P]  (middle right)
  |                                          |
  |  RED                              BLUE   |
  |  wall   [P P G]          [G P P]  wall   |
  |         (lower left)     (lower right)   |
  |            [RED box]       [BLUE box]    |
  |                                          |
  +------------------------------------------+
```

Key details:
- **Red alliance** starts top-right, scores toward right-side piles. **Blue** is mirrored top-left.
- **Levers** are horizontal bars on the side walls at mid-height (red on right, blue on left).
- **3 ball piles per side**, each with 3 balls (green=G, purple=P). Pile arrangements are fixed:
  - Upper: PPG | Middle: PGP | Lower: GPP
- **Scoring boxes** (red/blue outlined squares) are in the lower half of the field.
- The robot's autonomous starts at the red triangle (heading 45°), drives diagonally to the score position, then works the right-side piles top-to-bottom.

## Autonomous Status

See `AUTONOMOUS_TODO.md` (in TeamCode pedroPathing package) for the full gap analysis, corrected SPIT_ROT tables, and step-by-step completion plan.

## Dependencies

FTC SDK 11.0.0, Pedro Pathing 2.0.6 (`com.pedropathing:ftc`), ByLazar Panels 1.0.9 for telemetry visualization. Dependency versions are centralized in `build.dependencies.gradle`.
