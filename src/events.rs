use keyberon::layout::Event;

/// Deserialise a slice of bytes into a keyberon Event.
///
/// The serialisation format must be compatible with
/// the serialisation format in `ser`.
pub fn de(bytes: &[u8]) -> Result<Event, ()> {
    match *bytes {
        [b'P', i, j, b'\n'] => Ok(Event::Press(i, j)),
        [b'R', i, j, b'\n'] => Ok(Event::Release(i, j)),
        _ => Err(()),
    }
}

/// Serialise a keyberon event into an array of bytes.
///
/// The serialisation format must be compatible with
/// the serialisation format in `de`.
pub fn ser(e: Event) -> [u8; 4] {
    match e {
        Event::Press(i, j) => [b'P', i, j, b'\n'],
        Event::Release(i, j) => [b'R', i, j, b'\n'],
    }
}

#[cfg(feature = "right")]
pub fn tans(e:Event)->Event {
    match e {
        Event::Press(i, j) => Event::Press(i, j+5),
        Event::Release(i, j) => Event::Release(i, j+5),
    }
}

#[cfg(feature = "left")]
pub fn tans(e:Event)->Event {
    e
}

#[cfg(all(feature = "right", feature = "left"))]
compile_error!("feature \"left\" and feature \"right\" cannot be enabled at the same time");