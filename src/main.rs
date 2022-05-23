#![cfg_attr(debug_assertions, allow(dead_code, unused_imports, unused_mut, unused_variables, unreachable_code))]

extern crate kiddo;
extern crate prisma;
extern crate angular_units;

use std::env::current_dir;
use std::f32::consts::PI;
use std::path::Path;

use ggez::conf::{WindowMode, WindowSetup};
use ggez::event::{self, EventHandler, KeyCode, KeyMods};
use ggez::graphics::{self, Color, Font, Text, TextFragment};
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
    let mut window_setup = WindowSetup::default();
    window_setup.title = "Double pendulum fractal".into();
    let mut p = current_dir().unwrap();
    let (mut ctx, event_loop) = ContextBuilder::new("my_game", "Cool Game Author")
        .window_mode(window_mode)
        .window_setup(window_setup)
        .add_resource_path(&p)
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
    pos: Vec2,
}

impl TextHint {
    fn new(ctx: &mut Context) -> GameResult<Self> {
        Ok(TextHint {
            font: Font::new(ctx, "/OpenSans-Regular.ttf")?,
            text: None,
            pos: vec2(0.0, 0.0),
        })
    }

    fn draw(&self, ctx: &mut Context) -> GameResult<()> {
        if self.text.is_none() {
            return Ok(())
        }
        let text = Text::new(self.text.clone().unwrap());
        let mut pos = self.pos.clone();
        pos.y -= 20.0;
        pos.x += 10.0;
        graphics::draw(ctx, &text, (pos, Color::WHITE))?;
        Ok(())
    }
}

struct MyGame {
    pendulums: PendulumFamily,
    state: GameState,
    hint: TextHint,
}

impl MyGame {
    pub fn new(ctx: &mut Context) -> MyGame {
        let mut this = MyGame {
            pendulums: PendulumFamily::new(WIDTH),
            state: GameState::PAUSE,
            hint: TextHint::new(ctx).unwrap(),
        };
        //this.pendulums.add(DoublePendulum::new2(vec2(WIDTH / 2.0, WIDTH / 2.0), WIDTH, 1.0));
        this.pendulums.add(DoublePendulum::new2(vec2(WIDTH / 4.0, WIDTH / 4.0), WIDTH, 0.5));
        this.pendulums.add(DoublePendulum::new2(vec2(WIDTH / 4.0, WIDTH / 4.0 * 3.0 + 1.0), WIDTH, 0.5));
        this
    }
}

impl MyGame {
    fn update_hint(&mut self) {
        let (x, y) = (self.hint.pos.x, self.hint.pos.y);
        if let Some(ref p) = self.pendulums.find(x, y) {
            // self.hint.pos = vec2(x, y);
            self.hint.text = Some(format!("{}", p.borrow().steps / 1_000))
        }
        else {
            self.hint.text = None
        }
    }
}

impl EventHandler for MyGame {
    fn update(&mut self, ctx: &mut Context) -> GameResult<()> {
        self.update_hint();
        if self.state == GameState::PAUSE {
            return Ok(())
        }
        self.pendulums.update(ctx)?;
        if self.pendulums.len() == 0 && self.state == GameState::RUN {
            self.state = GameState::PAUSE;
        }
        Ok(())
    }
    
    fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
        graphics::clear(ctx, Color::WHITE);
        self.pendulums.draw(ctx)?;
        self.hint.draw(ctx)?;
        graphics::present(ctx)
    }

    fn key_down_event(&mut self, ctx: &mut Context, keycode: KeyCode, _keymods: KeyMods, _repeat: bool) {
        match keycode {
            KeyCode::Q => {
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
        self.hint.pos = vec2(x, y);
    }
}
