//! Little-endian reader and writer over byte slices, without allocation.

/// Why a frame could not be decoded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// The buffer ended before the frame did.
    Truncated,
    /// The frame kind byte is not known to this version of the protocol.
    UnknownKind(u8),
    /// The buffer length does not match the frame kind's fixed length.
    WrongLength,
}

/// Writes little-endian values into a byte slice.
pub struct Writer<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> Writer<'a> {
    /// Starts writing at the beginning of `buf`.
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Number of bytes written so far.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.pos
    }

    /// Returns `true` if nothing has been written.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.pos == 0
    }

    fn put(&mut self, bytes: &[u8]) {
        self.buf[self.pos..self.pos + bytes.len()].copy_from_slice(bytes);
        self.pos += bytes.len();
    }

    /// Writes one byte.
    pub fn u8(&mut self, value: u8) {
        self.put(&[value]);
    }

    /// Writes a `u16`.
    pub fn u16(&mut self, value: u16) {
        self.put(&value.to_le_bytes());
    }

    /// Writes a `u32`.
    pub fn u32(&mut self, value: u32) {
        self.put(&value.to_le_bytes());
    }

    /// Writes an `f32`.
    pub fn f32(&mut self, value: f32) {
        self.put(&value.to_le_bytes());
    }

    /// Writes three `f32`s.
    pub fn vec3(&mut self, value: [f32; 3]) {
        for v in value {
            self.f32(v);
        }
    }
}

/// Reads little-endian values from a byte slice.
pub struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    /// Starts reading at the beginning of `buf`.
    #[must_use]
    pub const fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    fn take<const N: usize>(&mut self) -> Result<[u8; N], DecodeError> {
        let end = self.pos + N;
        let bytes = self.buf.get(self.pos..end).ok_or(DecodeError::Truncated)?;
        self.pos = end;
        let mut out = [0; N];
        out.copy_from_slice(bytes);
        Ok(out)
    }

    /// Reads one byte.
    ///
    /// # Errors
    /// [`DecodeError::Truncated`] if the buffer is exhausted.
    pub fn u8(&mut self) -> Result<u8, DecodeError> {
        Ok(self.take::<1>()?[0])
    }

    /// Reads a `u16`.
    ///
    /// # Errors
    /// [`DecodeError::Truncated`] if the buffer is exhausted.
    pub fn u16(&mut self) -> Result<u16, DecodeError> {
        Ok(u16::from_le_bytes(self.take()?))
    }

    /// Reads a `u32`.
    ///
    /// # Errors
    /// [`DecodeError::Truncated`] if the buffer is exhausted.
    pub fn u32(&mut self) -> Result<u32, DecodeError> {
        Ok(u32::from_le_bytes(self.take()?))
    }

    /// Reads an `f32`.
    ///
    /// # Errors
    /// [`DecodeError::Truncated`] if the buffer is exhausted.
    pub fn f32(&mut self) -> Result<f32, DecodeError> {
        Ok(f32::from_le_bytes(self.take()?))
    }

    /// Reads three `f32`s.
    ///
    /// # Errors
    /// [`DecodeError::Truncated`] if the buffer is exhausted.
    pub fn vec3(&mut self) -> Result<[f32; 3], DecodeError> {
        Ok([self.f32()?, self.f32()?, self.f32()?])
    }
}
