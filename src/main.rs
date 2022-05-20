#![cfg_attr(debug_assertions, allow(dead_code, unused_imports, unused_mut, unused_variables, unreachable_code))]

#[macro_use]
extern crate lazy_static;

use std::f32::consts::PI;

use ggez::conf::{WindowMode, WindowSetup};
use ggez::event::{self, EventHandler, KeyCode, KeyMods};
use ggez::graphics::{self, Color, Font};
use ggez::{Context, ContextBuilder, GameResult};
use glam::*;
use pendulum::{DoublePendulum, PendulumFamily};

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

#[derive(PartialEq)]
enum GameState {
    PAUSE,
    RUN,
    DONE,
}

struct TextHint {
    font: Font,
    text: Option<String>,
}

impl TextHint {
    fn new(ctx: &mut Context) -> GameResult<Self> {
        Ok(TextHint {
            font: Font::new(ctx, "")?,
            text: None,
        })
    }
}

struct MyGame {
    // ps:    Vec<DoublePendulum>, // running
    // st:    Vec<DoublePendulum>, // stopped
    pendulums: PendulumFamily,
    state: GameState,
    hint: TextHint,
}

impl MyGame {
    pub fn new(ctx: &mut Context) -> MyGame {
        let mut this = MyGame {
            pendulums: PendulumFamily::new(),
            state: GameState::PAUSE,
            hint: TextHint::new(ctx).unwrap(),
        };
        this.pendulums.add(DoublePendulum::new2(vec2(WIDTH / 2.0, WIDTH / 2.0), WIDTH, 1.0));
        this
    }
}

impl EventHandler for MyGame {
    fn update(&mut self, ctx: &mut Context) -> GameResult<()> {
        if self.state == GameState::PAUSE {
            return Ok(())
        }
        self.pendulums.update(ctx, WIDTH)?;
        if self.pendulums.len() == 0 && self.state == GameState::RUN {
            self.state = GameState::PAUSE;
        }
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
        graphics::clear(ctx, Color::WHITE);
        self.pendulums.draw(ctx)?;
        graphics::present(ctx)
    }

    fn key_down_event(&mut self, ctx: &mut Context, keycode: KeyCode, _keymods: KeyMods, _repeat: bool) {
        match keycode {
            KeyCode::Q => {
                // let mut steps = self.st.iter().map(|p| p.steps).collect::<Vec<_>>();
                // steps.sort();
                // println!("{:?}", steps);
                event::quit(ctx);
            },
            KeyCode::Space => {
                if self.state == GameState::RUN {
                    self.state = GameState::PAUSE;
                }
                else if self.state == GameState::PAUSE {
                    self.state = GameState::RUN;
                }
            }
            _ => {}
        }
    }
    
    fn mouse_motion_event(&mut self, _ctx: &mut Context, x: f32, y: f32, _dx: f32, _dy: f32) {
        if let Some(p) = self.pendulums.find(x, y) {

        }
    }
}
