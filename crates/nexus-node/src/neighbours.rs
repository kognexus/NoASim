//! What a node knows about the joints next to it: their announced parameters and their
//! latest published state, with its age.

use nexus_proto::{Announce, Chain, NodeId, NodeState};

/// Which side of the node a neighbour is on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    /// Towards the base (line position − 1).
    Inner,
    /// Towards the tool (line position + 1).
    Outer,
}

/// One neighbour as seen from this node.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Neighbour {
    /// Its bus address.
    pub id: NodeId,
    /// Its announced nominal parameters.
    pub announce: Announce,
    /// Its latest state, if any has arrived.
    pub state: Option<NodeState>,
    /// Cycle stamped on that state by the neighbour.
    pub state_cycle: u32,
    /// Cycle at which that state arrived here.
    pub received_cycle: u32,
    /// Its latest gravity-chain data, if any.
    pub chain: Option<Chain>,
    /// Cycle stamped on that chain data by the neighbour.
    pub chain_cycle: u32,
}

impl Neighbour {
    /// Cycles since the neighbour wrote its latest state, as of `now`.
    #[must_use]
    pub const fn age(&self, now: u32) -> u32 {
        now.wrapping_sub(self.state_cycle)
    }

    /// The latest chain data if it is at most `max_age` cycles old at `now`.
    #[must_use]
    pub fn fresh_chain(&self, now: u32, max_age: u32) -> Option<&Chain> {
        self.chain.as_ref().filter(|_| now.wrapping_sub(self.chain_cycle) <= max_age)
    }

    /// The latest state if it is at most `max_age` cycles old at `now`.
    #[must_use]
    pub fn fresh_state(&self, now: u32, max_age: u32) -> Option<&NodeState> {
        self.state.as_ref().filter(|_| self.age(now) <= max_age)
    }
}

/// The node's inner and outer neighbours.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Neighbourhood {
    inner: Option<Neighbour>,
    outer: Option<Neighbour>,
}

impl Neighbourhood {
    /// The neighbour on `side`, if discovered.
    #[must_use]
    pub const fn get(&self, side: Side) -> Option<&Neighbour> {
        match side {
            Side::Inner => self.inner.as_ref(),
            Side::Outer => self.outer.as_ref(),
        }
    }

    /// Records an announcement if it comes from a node next to `own_position`.
    pub fn on_announce(&mut self, own_position: u8, source: NodeId, announce: &Announce) {
        let slot = match announce.position {
            p if p.checked_add(1) == Some(own_position) => &mut self.inner,
            p if own_position.checked_add(1) == Some(p) => &mut self.outer,
            _ => return,
        };
        *slot = Some(Neighbour {
            id: source,
            announce: *announce,
            state: None,
            state_cycle: 0,
            received_cycle: 0,
            chain: None,
            chain_cycle: 0,
        });
    }

    /// Records a neighbour's gravity-chain data.
    pub fn on_chain(&mut self, source: NodeId, chain: &Chain, chain_cycle: u32) {
        for neighbour in [&mut self.inner, &mut self.outer].into_iter().flatten() {
            if neighbour.id == source {
                neighbour.chain = Some(*chain);
                neighbour.chain_cycle = chain_cycle;
            }
        }
    }

    /// Records a neighbour's state; states from other nodes or older than the one held
    /// are ignored.
    pub fn on_state(&mut self, source: NodeId, state: &NodeState, state_cycle: u32, now: u32) {
        for neighbour in [&mut self.inner, &mut self.outer].into_iter().flatten() {
            let newer =
                neighbour.state.is_none() || state_cycle.wrapping_sub(neighbour.state_cycle) < u32::MAX / 2;
            if neighbour.id == source && newer {
                neighbour.state = Some(*state);
                neighbour.state_cycle = state_cycle;
                neighbour.received_cycle = now;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Neighbourhood, Side};
    use nexus_proto::{Announce, NodeId, NodeState};

    fn announce(position: u8) -> Announce {
        Announce { position, ..Announce::default() }
    }

    #[test]
    fn keeps_only_adjacent_nodes_and_their_latest_state() {
        let mut n = Neighbourhood::default();
        n.on_announce(3, NodeId(2), &announce(2));
        n.on_announce(3, NodeId(4), &announce(4));
        n.on_announce(3, NodeId(5), &announce(5));
        assert_eq!(n.get(Side::Inner).map(|x| x.id), Some(NodeId(2)));
        assert_eq!(n.get(Side::Outer).map(|x| x.id), Some(NodeId(4)));

        let state = NodeState { motor_angle: 1.0, ..NodeState::default() };
        n.on_state(NodeId(4), &state, 10, 11);
        n.on_state(NodeId(4), &NodeState::default(), 9, 12); // older: ignored
        n.on_state(NodeId(5), &NodeState::default(), 12, 12); // not a neighbour: ignored
        let outer = n.get(Side::Outer).unwrap();
        assert_eq!(outer.state.map(|s| s.motor_angle), Some(1.0));
        assert_eq!(outer.age(14), 4);
        assert!(outer.fresh_state(14, 4).is_some());
        assert!(outer.fresh_state(15, 4).is_none());
    }
}
