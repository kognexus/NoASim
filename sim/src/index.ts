/**
 * Simulation harness for the Nexus networked-actuator arm.
 *
 * The harness owns everything that is *not* actuator firmware: the MuJoCo plant,
 * the sensor models, the EtherCAT-like bus and the host. Actuator nodes only ever
 * receive their own sensor frame and the bus frames addressed to them.
 *
 * @packageDocumentation
 */

/** Bus cycle rate shared by the plant step and every node tick, in hertz. */
export const BUS_CYCLE_HZ = 4000;

/** Duration of one bus cycle, in seconds. */
export const BUS_CYCLE_S = 1 / BUS_CYCLE_HZ;
