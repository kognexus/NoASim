//! Bus frames exchanged between Nexus actuator nodes and the host.
//!
//! The crate is `no_std` so the same definitions compile for the actuator MCU, for
//! WebAssembly and for host-side tools. Frame layouts are added in NX-05.

#![no_std]
#![forbid(unsafe_code)]

/// Address of a node on the bus, assigned during discovery.
///
/// Address `0` is reserved for the host; actuator nodes are numbered from the base
/// outward, starting at `1`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct NodeId(pub u8);

impl NodeId {
    /// The host (bus master) address.
    pub const HOST: Self = Self(0);

    /// Returns `true` if this address belongs to the host rather than an actuator.
    #[must_use]
    pub const fn is_host(self) -> bool {
        self.0 == Self::HOST.0
    }
}

#[cfg(test)]
mod tests {
    use super::NodeId;

    #[test]
    fn host_address_is_zero() {
        assert!(NodeId::HOST.is_host());
        assert!(!NodeId(1).is_host());
    }
}
