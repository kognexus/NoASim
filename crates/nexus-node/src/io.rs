//! The node's mailboxes: frames in from the bus, frames out to the bus.
//!
//! Both are byte buffers holding a sequence of `[length: u16 LE][frame bytes]` records,
//! so they can live in the node's fixed I/O memory without allocation.

use nexus_proto::{DecodeError, Frame};

/// Frames delivered to the node this cycle.
#[derive(Clone, Copy, Debug)]
pub struct Inbox<'a> {
    bytes: &'a [u8],
}

impl<'a> Inbox<'a> {
    /// Wraps the `bytes` the bus wrote.
    #[must_use]
    pub const fn new(bytes: &'a [u8]) -> Self {
        Self { bytes }
    }

    /// Iterates over the frames; malformed frames yield an error and are skipped.
    #[must_use]
    pub const fn frames(&self) -> Frames<'a> {
        Frames { rest: self.bytes }
    }
}

/// Iterator over the frames of an [`Inbox`].
pub struct Frames<'a> {
    rest: &'a [u8],
}

impl Iterator for Frames<'_> {
    type Item = Result<Frame, DecodeError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.rest.len() < 2 {
            return None;
        }
        let len = usize::from(u16::from_le_bytes([self.rest[0], self.rest[1]]));
        let Some(record) = self.rest.get(2..2 + len) else {
            self.rest = &[];
            return Some(Err(DecodeError::Truncated));
        };
        self.rest = &self.rest[2 + len..];
        Some(Frame::decode(record))
    }
}

/// Frames the node sends this cycle.
pub struct Outbox<'a> {
    bytes: &'a mut [u8],
    len: usize,
    dropped: u32,
}

impl<'a> Outbox<'a> {
    /// An empty outbox writing into `bytes`.
    pub fn new(bytes: &'a mut [u8]) -> Self {
        Self { bytes, len: 0, dropped: 0 }
    }

    /// Bytes used.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Returns `true` if no frame was sent.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Frames that did not fit.
    #[must_use]
    pub const fn dropped(&self) -> u32 {
        self.dropped
    }

    /// Queues `frame`; returns `false` (and counts a drop) if it does not fit.
    pub fn push(&mut self, frame: &Frame) -> bool {
        let n = frame.len();
        if self.len + 2 + n > self.bytes.len() {
            self.dropped += 1;
            return false;
        }
        #[allow(clippy::cast_possible_truncation)]
        let prefix = (n as u16).to_le_bytes();
        self.bytes[self.len..self.len + 2].copy_from_slice(&prefix);
        frame.encode(&mut self.bytes[self.len + 2..]);
        self.len += 2 + n;
        true
    }
}
