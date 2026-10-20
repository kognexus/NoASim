//! Bus frames exchanged between Nexus actuator nodes and the host.
//!
//! The crate is `no_std` and allocation-free so the same definitions compile for the
//! actuator MCU, for WebAssembly and for host-side tools.
//!
//! Every frame is a fixed-layout, little-endian byte string: an 8-byte [`Header`]
//! followed by a payload whose length is fixed per [`FrameKind`]. The layout is pinned by
//! `golden/frames.txt`, which both this crate's tests and the TypeScript codec's tests
//! check against, so the two implementations cannot drift apart.

#![no_std]
#![forbid(unsafe_code)]

mod codec;
mod frames;

pub use codec::{DecodeError, Reader, Writer};
pub use frames::{
    Announce, Assign, Chain, DampingMode, Destination, Frame, FrameKind, Goal, Header, Limits, NodeState,
    Settings, StatusFlags, HEADER_LEN, MAX_FRAME_LEN,
};

/// Address of a node on the bus, assigned during discovery.
///
/// Address `0` is reserved for the host; actuator nodes are numbered from the base
/// outward, starting at `1`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct NodeId(pub u8);

impl NodeId {
    /// The host (bus master) address.
    pub const HOST: Self = Self(0);
    /// Address of a node that has not been assigned one yet.
    pub const UNASSIGNED: Self = Self(0xFF);

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
