# Limitations

What this repository shows and what it does not. Numbers are in [RESULTS.md](RESULTS.md).

## What it is

- **A simulation.** It shows that the control architecture works. It does not show that
  the firmware meets microcontroller timing: the node code is `no_std`, allocation-free and
  kept MCU-shaped, but it runs as WebAssembly on a PC here.
- **The plant is lumped.** QDD joints are a motor hinge in series with a gear spring; links
  bend at one point each. The modes are tuned to realistic values but this is not a finite
  element model.
- **The actuators nobody described are still models we wrote.** Five open families and one
  sealed family differ in structure (friction laws, cogging and ripple, nonlinear and
  hysteretic gears, backlash, thermal drift). The learned-model results are only as
  convincing as those differences; real bench data should replace them.

## Targets the measurements miss

- **Network damping (H1).** It settles a tap about 2–2.5× faster than joint control alone,
  far from the 10× target. Against joint-local damping it is about even, and slower in some
  postures and seeds the tuner never saw, so the 3× target is missed outright. The targets
  assumed nearly undamped structural modes; a tuned joint servo already damps them well,
  and joint-local damping on the joint's own encoders gets most of what is left. What the
  network does buy is that speed at about a sixth of joint-local damping's rest jitter,
  because a gyro needs no differentiation (NX-11), and a damping term whose energy is
  bounded by a tank on measured power (H6).
- **Joint-level interpolation (H2).** Path error is better in every task, but tool jerk
  falls by 1–3×, not the 5× target. The arm's own servo bandwidth, not interpolation,
  limits how smooth the tool can be.
- **Attribution against joints alone (H7).** Each joint finds its own disturbances from
  its motor-side residual, which a joint alone can do just as well. On datasheet
  actuators the network therefore makes no fewer errors than joints alone (neither makes
  any). Naming the source from felt vibration, the original idea, named the wrong joint
  most of the time: the KX-6 joints share one fingerprint, and the arm's own modes ring at
  their order frequencies. The network's part is the corroboration, the correction and
  the fault.
- **Gravity from each joint's IMU (H9).** Shoulder and elbow are within 1 % of rated
  torque. The base joint reads a 20 mg accelerometer bias as a 1.1° tilt; the wrist joints
  assume the payload sits on the flange. Both need commissioning steps not built here
  (accelerometer offsets, the payload's centre of mass).
- **The learned model on the sealed family (H10a).** About 60 % of the gap to the truth
  closed against a 70 % target, and two of six joints end worse than the datasheet. LuGre
  friction has an internal state the model lacks, and one commissioning run covers only
  part of the eccentric gear.
- **Coordinated damping needs a clean bus and the arm it was designed for (NX-30).** In
  the joints it holds a tap 1.3–2.8× closer than network damping, but only while the
  arm holds still and every frame it needs is at most 4 cycles old. On a bus losing a
  fifth of its frames it falls back to network damping and gains nothing, although the
  same estimator outside the joints keeps its advantage there. The design is made
  offline, for one arm and payload at one posture; with a 3 kg payload it was not made
  for, the stiff arm held the tool 0.6 mm off after a tap. A joint cannot design or
  check it itself.
- **Modes no joint reaches.** On the light arm, link bending at 55–69 Hz bends across
  every joint axis; no joint's torque or sensors reach it, so no damping in the joints,
  learned or not, can touch it. Only the structure's own damping does (NX-28).
- **Budget actuators with gear play.** A joint estimates the torque it holds from its
  gear twist through its own model of the gear. With the datasheet model, play reads as
  load: on budget actuators the payload estimate was four times too high and the arm
  stood 1° low (25 mm tracking error). With the learned model it is 5 mm, but the
  payload still reads 60 % high and attribution is 6 of 10 (NX-26). The grades' levels
  are our estimates, not measured products.
- **Joint-local damping under a classic controller.** During a classic controller's
  pick and place, joint-local damping makes the outer joints vibrate up to 13 % more than
  no damping and tracks 0.2 mm worse (measured in NX-25, unchanged by it). Likely cause,
  not yet checked: a joint's own gyro also turns with the joints inside it, and alone it
  cannot tell their commanded motion from vibration. Network damping takes the difference
  of two gyros and does not show it.


## Things the design deliberately does not do

- **Impact detection is an early-warning layer.** Speed and force limits remain the
  primary protection (Haddadin 2009, ISO/TS 15066). A 200 N tap at the tool is an impact
  to the joints; they cannot tell it from a collision.
- **Without brakes, a QDD arm has no safe unpowered state.** Safe torque off drops it
  under gravity. Retreat and hold keeps it powered; brakes are the only answer to power
  loss.
- **Retreating to the last correct state** assumes the space the arm came through is
  still free. The simulation checks speed limits, not obstacles that moved in behind it.
- **Gravity from the IMU is solved joint by joint only for an open chain.** In contact or
  a closed chain it stays a feed-forward estimate offered to the host.
- **Nothing learned is in the safety path.** The damping law's energy tank uses measured
  power. A wrong learned model can still degrade performance (its feed-forward is in the
  loop); it cannot inject energy through the damping term.
- **A worn bearing is not attributed.** Its pulses are incoherent, so neither its own
  residual nor its neighbours see an angle-locked component. Anything coherent enough to be
  attributed can also be cancelled, so the "cannot fix it" case in the proof is a ripple
  beyond the drive's compensation budget.

## The demo

- The page steps the simulation for at most 14 ms per frame and shows the real-time
  factor; two arms run at about 2× real time on a laptop in Node, less in a browser.
- A browser tab that is not visible stops animation frames, and the simulation with them.
- Arm editing is limited to the body (KX-6 or KX-6L), payload, link stiffness and brakes;
  the number of joints and link lengths are fixed.
