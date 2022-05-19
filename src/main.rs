#![cfg_attr(debug_assertions, allow(dead_code, unused_imports, unused_mut, unused_variables, unreachable_code))]

use std::f32::consts::PI;

use ggez::conf::{WindowSetup, WindowMode};
use ggez::event::{self, EventHandler, KeyCode, KeyMods};
use ggez::graphics::{self, Color};
use ggez::{Context, ContextBuilder, GameResult};
use glam::*;
use pendulum::DoublePendulum;

mod pendulum;

const PHI: f32 = PI * 2.0;
const WIDTH: f32 = 2048.0;

fn main() {
    // Make a Context.
    let mut window_mode = WindowMode::default();
    window_mode.width = WIDTH;
    window_mode.height = WIDTH;
    let (mut ctx, event_loop) = ContextBuilder::new("my_game", "Cool Game Author")
        .window_mode(window_mode)
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
    ps: Vec<DoublePendulum>, // running
    st: Vec<DoublePendulum>, // stopped
}

impl MyGame {
    pub fn new(_ctx: &mut Context) -> MyGame {
        MyGame {
            ps: vec![
                DoublePendulum::new2(vec2(WIDTH/2.0, WIDTH/2.0), WIDTH, 1.0),
            ],
            st: vec![],
        }
    }
}

impl EventHandler for MyGame {
    fn update(&mut self, ctx: &mut Context) -> GameResult<()> {
        let mut p1 = Vec::new();
        for p in &mut self.ps {
            p.update(ctx)?;
            if p.stopped {
                self.st.push(p.clone());
                p1.extend(p.split(WIDTH));
                println!("{:?} {:?}", p, p1);
            }
            else {
                p1.push(p.clone())
            }
        }
        self.ps = p1;
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
        graphics::clear(ctx, Color::WHITE);
        for p in &self.st {
            p.draw(ctx)?
        }
        for p in &mut self.ps {
            p.draw(ctx)?
        }
        graphics::present(ctx)
    }

    fn key_down_event(&mut self, ctx: &mut Context, keycode: KeyCode, _keymods: KeyMods, _repeat: bool) {
        match keycode {
            KeyCode::Q => event::quit(ctx),
            _ => {}
        }
    }
}
