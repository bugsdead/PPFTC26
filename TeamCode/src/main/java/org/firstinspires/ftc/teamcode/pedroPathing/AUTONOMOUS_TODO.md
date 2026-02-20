# FTC Autonomous Completion - Summary & TODO List

## Context
FTC DECODE 2025-2026. Robot has a 3-slot revolver that intakes balls, sorts by rotating to the correct slot, and launches via gecko wheels + ball pusher. Each match reveals a **randomized target launch sequence** (GPP, PGP, or PPG) — the driver selects the matching autonomous file.

## Key Facts
- **Pile arrangements are FIXED every match:**
  - Upper right: PPG (slot0=P, slot1=P, slot2=G)
  - Middle: PGP (slot0=P, slot1=G, slot2=P)
  - Lower right: GPP (slot0=G, slot1=P, slot2=P)
- **Target launch sequence is RANDOMIZED** — determines which file to run
- **Red lever** on right side of field must be hit (drive-through, no stop needed)
- **30-second autonomous** — aim for 2 piles + lever + position for pile 3
- **Starting position:** Red triangle (top-right), heading 45°

## Autonomous Flow (all 3 files)
1. Drive to SCORE_POS (23.18, 30.76), heading 45°→231° — spit pre-loaded balls
2. Drive to PUSH_START (0.88, 34.01) → drive to pile 1 (upper right) → intake 3 balls
3. Drive to SCORE_POS → spit pile 1 balls in target sequence
4. Drive to pile 2 (middle) → intake 3 balls
5. Drive through red lever (on the way back) → drive to SCORE_POS → spit pile 2 balls
6. If time: position near pile 3 (lower right) for teleop

## Current State: What's Working vs What's Not

**In practice, the autonomous currently does**: Drive to score position → spit 3 pre-loaded balls → drive to push start → stop. That's ~2 paths out of ~12 needed per match.

### Working (tested & confirmed)
- SCORE_POS (23.18, 30.76) — launch position confirmed
- pathToDeposit — start→SCORE_POS with heading 45°→231°
- Spit sequence — gecko overlap with drive, per-ball power (0.49/0.42/0.45), push 0.5s/0.4s/0.4s
- PUSH_START (0.88, 34.01) — 2nd path heading 231°→180° (direction OK, distance tuning needed)
- Voltage compensation on gecko motors
- State machine framework in GPP/PGP/PPG — logic is sound, flow is correct
- SPIT_ROT values and comments — corrected in all 3 files (GPP, PGP, PPG)
- TICKS_PER_REV = 448.1

### Built but broken/incomplete
- **All pile coordinates** — placeholders `(-20,40)` etc., need field measurement
- **Red lever path** — not implemented in any file
- ~~**Time management**~~ — added 25s threshold in `afterSpit()`, positions near pile 3 for teleop if late
- **Intake direction** — untested in autonomous (`+1.0` in auto vs `-1.0` in TestIntakeSequence)
- ~~**FTCAutonomous2026 intake pattern**~~ — fixed to 120/120/120/30° with motor busy guards

## CORRECTED Spit Rotation Tables
Revolver: 120° = next slot, 240° = skip one slot

**FTCAutonomousGPP.java** (target: G, P, P):
```
Pre-loaded:           {0, 240, 240}   // Standard (tested)
Pile 1 - upper PPG:   {240, 120, 120} // G(slot2), P(slot0), P(slot1)
Pile 2 - middle PGP:  {120, 120, 120} // G(slot1), P(slot2), P(slot0)
Pile 3 - lower GPP:   {0, 120, 120}   // G(slot0), P(slot1), P(slot2) - natural
```

**FTCAutonomousPGP.java** (target: P, G, P):
```
Pre-loaded:           {0, 240, 240}   // Standard (tested)
Pile 1 - upper PPG:   {0, 240, 240}   // P(slot0), G(slot2), P(slot1)
Pile 2 - middle PGP:  {0, 120, 120}   // P(slot0), G(slot1), P(slot2) - natural
Pile 3 - lower GPP:   {120, 240, 240} // P(slot1), G(slot0), P(slot2)
```

**FTCAutonomousPPG.java** (target: P, P, G):
```
Pre-loaded:           {0, 240, 240}   // Standard (tested)
Pile 1 - upper PPG:   {0, 120, 120}   // P(slot0), P(slot1), G(slot2) - natural
Pile 2 - middle PGP:  {0, 240, 240}   // P(slot0), P(slot2), G(slot1)
Pile 3 - lower GPP:   {120, 120, 120} // P(slot1), P(slot2), G(slot0)
```

### ~~Diffs applied~~ — SPIT_ROT values and comments are now correct in all 3 files as of step 1 completion.

## TODO List

### ~~1. Fix SPIT_ROT values and comments in all 3 files~~ DONE
- [x] Update FTCAutonomousGPP.java SPIT_ROT pile rows to corrected values
- [x] Update FTCAutonomousPGP.java SPIT_ROT pile rows to corrected values
- [x] Update FTCAutonomousPPG.java SPIT_ROT pile rows to corrected values
- [x] Fix SPIT_ROT comments in all 3 files — label piles as upper=PPG, middle=PGP, lower=GPP

### 2. Fine-tune PUSH_START
- [ ] Test and adjust PUSH_START coordinates on field (direction confirmed, distance needs tuning)

### 3. Measure and set pile coordinates (requires field access — biggest blocker)
- [ ] Measure pile 1 (upper right PPG) approach + end positions on field
- [ ] Measure pile 2 (middle PGP) approach + end positions
- [ ] Measure pile 3 (lower right GPP) approach + end positions
- [ ] Update PILE_1/2/3_APPROACH and _END in all 3 files
- [ ] Ensure intake paths (approach → end) are 15"+ to avoid Pedro Pathing oscillation

### 4. Test intake sequence (requires robot)
- [ ] Verify rIntake direction: autonomous uses `+1.0`, TestIntakeSequence uses `-1.0` — which is correct?
- [ ] Verify intake rotation pattern (120°, 120°, 120°, 30°) works during driving
- [ ] Tune BALL_INTAKE_TIME (currently 1.0s)

### 5. Test one file end-to-end for pile 1
- [ ] Test drive-to-pile path only (with real coordinates)
- [ ] Test intake at pile 1
- [ ] Test full cycle: drive → intake → return → spit for pile 1
- [ ] Verify 30° alignment rotation after intake positions correctly for spit
- [ ] **WATCH FOR**: SPIT_ROT cumulative position issue (see note below)

> **Code review finding — SPIT_ROT cumulative position concern:**
> The pile SPIT_ROT rows (indices 1-3) were computed as if the revolver starts at pos=0° for each pile spit.
> But the revolver position is cumulative: pre-loaded spit `{0,240,240}` ends at 120° (mod 360), then
> intake adds 120+120+120+30=390° more, leaving the revolver at 150° (mod 360) — NOT slot-aligned.
> The first rotation for each pile row may land between slots instead of on one.
>
> **If the first ball of pile 1 fires wrong/misaligned during testing**, the fix is to adjust
> `SPIT_ROT[pile][0]` to account for the revolver's actual position after intake. The within-row
> chaining (columns 1 and 2) is correct — only column 0 of each pile row would need adjustment.
>
> This only affects piles 1-3 (pre-loaded is tested and works). It cannot be verified until pile
> coordinates are real and the full intake→spit cycle runs on the field.

### 6. Add red lever hit (requires field measurement)
- [ ] Measure red lever position on field
- [ ] Replace pathToScore[1] (pile 2 → SCORE_POS) with multi-segment path routing through lever
- [ ] Tune lever drive-through heading

### 7. Test pile 2 full cycle
- [ ] Test drive → intake → lever → score for pile 2
- [ ] Verify lever hit is reliable without stopping

### ~~8. Add time management for pile 3~~ DONE
- [x] Add time check in `afterSpit()` before attempting pile 3 (`getRuntime() >= 25` threshold)
- [x] Add fallback: if insufficient time, drive toward pile 3 approach for teleop instead of full cycle

### 9. Tune path headings (iterative, on-field)
- [ ] pathToPile headings for each pile (intake orientation)
- [ ] pathToScore headings for returns
- [ ] Lever drive-through heading

### 10. Test all 3 sequence files
- [ ] Test GPP against all 3 pile arrangements
- [ ] Test PGP against all 3 pile arrangements
- [ ] Test PPG against all 3 pile arrangements

### 11. Blue alliance autonomous (future)
- [ ] Mirror all 3 files for blue alliance (top-left start)
- [ ] Mirror coordinates and headings
- [ ] Hit blue lever instead of red

### 12. Code cleanup (low priority)
- [x] Remove unused INTAKE_BAND_OFFSET from FTCAutonomous2026
- [ ] Remove unused BezierCurve import from FTCAutonomous2026
- [x] Fix FTCAutonomous2026 intake pattern: 30/60/60/30° → 120/120/120/30° (also added `!hw.revolverMotor.isBusy()` guards to match sequence files)
- [x] Remove ~50 lines of commented-out paths from FTCAutonomous2026
- [ ] Sync gecko power constants across files

## Testing Order
1. ~~Fix SPIT_ROT values in all 3 files~~ DONE
2. ~~Fix SPIT_ROT comments~~ DONE
3. Finish tuning PUSH_START on field
4. Measure pile coordinates on field, update all 3 files
5. Test rIntake direction on robot
6. Test one file (e.g., PGP) drive-to-pile path only
7. Test intake at pile 1
8. Test full cycle: drive → intake → return → spit for pile 1
9. Add and test lever hit on return from pile 2
10. Test pile 2 full cycle
11. ~~Add time check for pile 3~~ DONE
12. Test other sequence files

## Tested Constants Reference
- TICKS_PER_REV: 448.1
- REVOLVER_POWER: 0.43
- GECKO_POWER_1: 0.49 (1st ball)
- GECKO_POWER_2: 0.42 (2nd ball)
- GECKO_POWER_3: 0.45 (3rd ball)
- Gecko startup delay: 0.8s (overlapped with driving)
- Ball push duration: 0.5s first, 0.4s subsequent
- Revolver between balls (spit): 240° (for pre-loaded, varies per pile/target)
- Revolver between balls (intake): 120° + 30° final alignment
- Ball pusher: 0.0 = extended, 0.5 = retracted
- Intake: rIntake at 1.0 power (CRServo)
- Voltage compensation: 12V nominal / actual voltage

## Files
- FTCAutonomous2026.java — general/test autonomous (2 paths working, intake pattern outdated)
- FTCAutonomousGPP.java — target sequence G,P,P (SPIT_ROT fixed)
- FTCAutonomousPGP.java — target sequence P,G,P (SPIT_ROT fixed)
- FTCAutonomousPPG.java — target sequence P,P,G (SPIT_ROT fixed)
- FTChardware.java — hardware abstraction (voltage compensation, gecko, revolver, servos)
- Constants.java — Pedro Pathing config (PID, velocities, constraints)
- TestIntakeSequence.java — intake test OpMode
- TestSpitOutBall.java — spit test OpMode
- FTCTeleOp2026.java — driver control
