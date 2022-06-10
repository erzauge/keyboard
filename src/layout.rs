
use keyberon::key_code::KeyCode::*;



#[rustfmt::skip]
pub static LAYERS:keyberon::layout::Layers<10, 4, 1, ()> = keyberon::layout::layout! {
    {
        [Q W E R T Y U I O P],
        [A S D F G H J K L Quote],
        [Z X C V B N M Comma Dot Bslash],
        [n n Escape Space Tab Enter Space BSpace n n ]
    }
};