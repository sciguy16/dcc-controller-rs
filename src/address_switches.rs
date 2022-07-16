// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, you can obtain one at https://mozilla.org/MPL/2.0/.

use embedded_hal::digital::v2::InputPin;

/// Convert a 7-bit dipswitch input to a u8 address value
pub struct AddressSwitches<
    A: InputPin,
    B: InputPin,
    C: InputPin,
    D: InputPin,
    E: InputPin,
    F: InputPin,
    G: InputPin,
> {
    a: A,
    b: B,
    c: C,
    d: D,
    e: E,
    f: F,
    g: G,
    value: u8,
}

impl<
        A: InputPin,
        B: InputPin,
        C: InputPin,
        D: InputPin,
        E: InputPin,
        F: InputPin,
        G: InputPin,
    > AddressSwitches<A, B, C, D, E, F, G>
{
    pub fn new(a: A, b: B, c: C, d: D, e: E, f: F, g: G) -> Self {
        let mut addr = Self {
            a,
            b,
            c,
            d,
            e,
            f,
            g,
            value: 0,
        };
        addr.update();
        addr
    }

    pub fn update(&mut self) {
        match (
            self.a.is_low(),
            self.b.is_low(),
            self.c.is_low(),
            self.d.is_low(),
            self.e.is_low(),
            self.f.is_low(),
            self.g.is_low(),
        ) {
            (Ok(a), Ok(b), Ok(c), Ok(d), Ok(e), Ok(f), Ok(g)) => {
                self.value = a as u8
                    | (b as u8) << 1
                    | (c as u8) << 2
                    | (d as u8) << 3
                    | (e as u8) << 4
                    | (f as u8) << 5
                    | (g as u8) << 6
            }
            _ => panic!(),
        }
    }

    pub fn value(&self) -> u8 {
        self.value
    }
}
