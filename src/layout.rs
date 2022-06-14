
use keyberon::key_code::KeyCode::*;

use keyberon::action::*;

const SPACE_1 : Action<()> =Action::HoldTap { timeout: 200, hold: &Action::Layer(1), tap: &k(Space), config: HoldTapConfig::HoldOnOtherKeyPress, tap_hold_interval: 0 };
const TAB_CTRL : Action<()> =Action::HoldTap { timeout: 200, hold: &k(LCtrl), tap: &k(Tab), config: HoldTapConfig::HoldOnOtherKeyPress, tap_hold_interval: 0 };
const ESCAPE_ALT : Action<()> =Action::HoldTap { timeout: 200, hold: &k(LAlt), tap: &k(Escape), config: HoldTapConfig::HoldOnOtherKeyPress, tap_hold_interval: 0 };
const Z_SHIFT : Action<()> =Action::HoldTap { timeout: 200, hold: &k(LShift), tap: &k(Z), config: HoldTapConfig::HoldOnOtherKeyPress, tap_hold_interval: 0 };
const BSLASH_SHIFT : Action<()> =Action::HoldTap { timeout: 200, hold: &k(RShift), tap: &k(Bslash), config: HoldTapConfig::HoldOnOtherKeyPress, tap_hold_interval: 0 };
const SLASH_SHIFT : Action<()> =Action::HoldTap { timeout: 200, hold: &k(RShift), tap: &k(Slash), config: HoldTapConfig::HoldOnOtherKeyPress, tap_hold_interval: 0 };

#[rustfmt::skip]
pub static LAYERS:keyberon::layout::Layers<10, 4, 2, ()> = keyberon::layout::layout! {
    {
        [Q W E R T Y U I O P],
        [A S D F G H J K L Quote],
        [{Z_SHIFT} X C V B N M Comma Dot {SLASH_SHIFT}],
        [n n {ESCAPE_ALT} LGui {TAB_CTRL} Enter {SPACE_1} BSpace n n ]
    }
    {
        [1 2 3 4 5 6 7 8 9 0],
        [Grave t t t t Left Down Up Right SColon],
        [t t t t t Minus Equal LBracket RBracket {BSLASH_SHIFT}],
        [t t t t t t t t t t]
    }
};