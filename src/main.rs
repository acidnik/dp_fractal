#![cfg_attr(debug_assertions, allow(dead_code, unused_imports, unused_mut, unused_variables, unreachable_code))]

use std::f32::consts::PI;

use ggez::event::{self, EventHandler};
use ggez::graphics::{self, Color};
use ggez::{Context, ContextBuilder, GameResult};
use glam::*;
use pendulum::DoublePendulum;

mod pendulum;

const PHI: f32 = PI * 2.0;

fn main() {
    // Make a Context.
    let (mut ctx, event_loop) = ContextBuilder::new("my_game", "Cool Game Author")
        .build()
        .expect("aieee, could not create ggez context!");

    // Create an instance of your event handler.
    // Usually, you should provide it with the Context object to
    // use when setting your game up.
    let my_game = MyGame::new(&mut ctx);

    // Run!
    event::run(ctx, event_loop, my_game);
}

struct MyGame {
    // Your state here...
    p: DoublePendulum,
}

impl MyGame {
    pub fn new(_ctx: &mut Context) -> MyGame {
        // Load/create resources such as images here.
        MyGame {
            p: DoublePendulum {
                p:      vec2(100.0, 100.0),
                theta1: PHI / 8.0,
                theta2: PHI / 4.0,
                l1:     100.0,
                l2:     150.0,
                dt1:    0.0,
                dt2:    0.0,
                d2t1: 0.0,
                d2t2: 0.0,
            },
        }
    }
}

impl EventHandler for MyGame {
    fn update(&mut self, ctx: &mut Context) -> GameResult<()> {
        self.p.update(ctx)?;
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
        graphics::clear(ctx, Color::WHITE);
        self.p.draw(ctx)?;
        graphics::present(ctx)
    }
}
