#![cfg_attr(debug_assertions, allow(dead_code, unused_imports, unused_mut, unused_variables, unreachable_code))]

extern crate angular_units;
extern crate prisma;
extern crate tap;

use std::env::current_dir;
use std::f64::consts::{TAU, PI};
use std::path::Path;
use std::time::{Duration, Instant};

use ggez::conf::{WindowMode, WindowSetup, ModuleConf};
use ggez::event::{self, EventHandler, KeyCode, KeyMods};
use ggez::graphics::{self, Canvas, Color, Font, Text, TextFragment, Rect};
use ggez::{timer, Context, ContextBuilder, GameResult};
use glam::*;
use p2::{PendulumFamily2};
use pendulum::{DoublePendulum, PendulumFamily, Config};
use tap::Tap;

mod pendulum;
mod avgspeed;
mod p2;

const WIDTH: f64 = 2048.0;

fn main() {
    // Make a Context.
    let mut window_mode = WindowMode::default();
    window_mode.width = WIDTH as f32;
    window_mode.height = WIDTH as f32;
    
    let window_setup = WindowSetup::default().tap_mut(|x| x.title="Double pendulum fractal".into());

    let module_conf = ModuleConf::default().audio(false);

    let p = current_dir().unwrap();
    let (mut ctx, event_loop) = ContextBuilder::new("my_game", "Cool Game Author")
        .window_mode(window_mode)
        .window_setup(window_setup)
        .modules(module_conf)
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
    pos:  Vec2,
}

impl TextHint {
    fn new(ctx: &mut Context) -> GameResult<Self> {
        Ok(TextHint {
            font: Font::new(ctx, "/OpenSans-Regular.ttf")?,
            text: None,
            pos:  vec2(0.0, 0.0),
        })
    }

    fn draw(&self, ctx: &mut Context) -> GameResult<()> {
        if self.text.is_none() {
            return Ok(());
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
    state:     GameState,
    hint:      TextHint,
}

impl MyGame {
    pub fn new(ctx: &mut Context) -> MyGame {
        // let config = Config {
        //     xmin: 0.0,
        //     xmax: TAU,
        //     ymin: 0.0,
        //     ymax: PI,
        //     color_step: 100.0,
        //     dive_diff: 0.990,
        //     max_step: 50_000,
        //     min_pixel: 4.0,
        //     color_mod: 6000,
        //     speed_a: 550.0,
        //     speed_b: 20.0,
        // };
        let config = Config {
            xmin: 0.0,
            xmax: TAU,
            ymin: 0.0,
            ymax: PI,
            color_step: 100.0,
            dive_diff: 0.82,
            max_step: 460_000,
            min_pixel: 4.0,
            color_mod: 6000,
            speed_a: 550.0,
            speed_b: 20.0,
        };
        // the eye
        // let config = Config {
        //     xmin: 1.24,
        //     xmax: 0.405,
        //     ymin: 1.45,
        //     ymax: 0.385,
        //     color_step: 100.0,
        //     dive_diff: 0.97,
        //     max_step: 1_000_000,
        //     min_pixel: 8.0,
        //     color_mod: 6_000,
        //     speed_a: 550.0,
        //     speed_b: 20.0,
        // };
        // eye + more context
        // let config = Config {
        //     xmin: 0.56,
        //     xmax: 1.1,
        //     ymin: 1.01,
        //     ymax: 0.8,
        //     color_step: 250.0,
        //     dive_diff: 0.999,
        //     max_step: 3_000_000,
        //     min_pixel: 4.0,
        //     // color_mod: 30700,
        //     color_mod: 1200700,
        //     speed_a: 620.0,
        //     speed_b: 20.0,
        // };
        // let config = Config {
        //     xmin: 5.09,
        //     xmax: 0.25,
        //     ymin: 1.6,
        //     ymax: 0.25,
        //     color_step: 2000.0,
        //     dive_diff: 0.994,
        //     max_step: 10_000,
        //     min_pixel: 4.0,
        //     color_mod: 100_000,
        // };

        // let config = p2::Config {
        //     xmin: 0.0,
        //     xmax: TAU,
        //     ymin: 0.0,
        //     ymax: PI,
        //     min_pixel: 8.0,
        //     update_steps: 10,
        // };
        
        let mut this = MyGame {
            pendulums: PendulumFamily::new(ctx, config.clone(), WIDTH),
            // pendulums: PendulumFamily2::new(ctx, config.clone(), WIDTH),
            state:     GameState::PAUSE,
            // state: GameState::RUN,
            hint:      TextHint::new(ctx).unwrap(),
        };
        //this.pendulums.init();
        this.pendulums.add(DoublePendulum::new2(dvec2(WIDTH / 2.0, WIDTH / 2.0), WIDTH, 1.0, &config));
        // this.pendulums.add(DoublePendulum::new2(vec2(768.0, 768.0), WIDTH, 0.25));
        // this.pendulums.add(DoublePendulum::new2(vec2(WIDTH / 4.0, WIDTH / 4.0 * 3.0), WIDTH, 0.5));
        // for _ in 1..2300 {
        //     this.pendulums.update(ctx).unwrap();
        // }
        // for p in this.pendulums.ps.values() {
        //     let p = p.borrow();
        //     println!("{} {}", p.id, p.stopped);
        // }
        // this.pendulums.add(DoublePendulum::new2(vec2(WIDTH / 4.0 * 3.0, WIDTH / 4.0), WIDTH, 0.5));
        this
    }
}

impl MyGame {
}

impl EventHandler for MyGame {
    fn update(&mut self, ctx: &mut Context) -> GameResult<()> {
        // self.update_hint();
        if self.state == GameState::PAUSE {
            return Ok(());
        }
        self.pendulums.update(ctx)?;
        // let img = graphics::screenshot(ctx).unwrap();
        // img.encode(ctx, graphics::ImageFormat::Png, format!("/{:06}.png", self.pendulums.iter)).unwrap();
        if self.pendulums.len() == 0 && self.state == GameState::RUN {
            println!("===END===");
            self.state = GameState::PAUSE;
        }
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
        let t = Instant::now();
        self.pendulums.draw(ctx)?;
        // self.hint.draw(ctx)?;
        let res = graphics::present(ctx);
        // println!("draw: {:?}", t.elapsed());
        res
    }

    fn key_down_event(&mut self, ctx: &mut Context, keycode: KeyCode, _keymods: KeyMods, _repeat: bool) {
        match keycode {
            KeyCode::Q => {
                event::quit(ctx);
            }
            KeyCode::Equals => {
                self.pendulums.update_steps *= 2;
            }
            KeyCode::Space => {
                if self.state == GameState::RUN {
                    println!("===PAUSE===");
                    self.state = GameState::PAUSE;
                }
                else if self.state == GameState::PAUSE {
                    println!("===RUN===");
                    self.state = GameState::RUN;
                }
            }
            _ => {}
        }
    }

    fn mouse_motion_event(&mut self, _ctx: &mut Context, x: f32, y: f32, _dx: f32, _dy: f32) {
        self.hint.pos = vec2(x, y);
    }

    fn mouse_button_down_event(&mut self, _ctx: &mut Context, button: event::MouseButton, x: f32, y: f32) {
        let p = self.pendulums.find_all(x as f64, y as f64);
        if let Some(pref) = p {
            let stopped;
            {
                let p = pref.borrow();
                println!(
                    "[{}] ({}, {}) sc={} st={} {} dive={} run={} ngs={:?}",
                    p.id,
                    p.p.x,
                    p.p.y,
                    p.scale,
                    p.stopped,
                    p.steps,
                    self.pendulums.dive.contains(&p.id),
                    self.pendulums.ps.contains_key(&p.id),
                    p.neighbors,
                );
                stopped = p.stopped;
            }

            if stopped && button == event::MouseButton::Left {
                // let mut ps = vec![pref.clone()];
                // let next = self.pendulums.dive_all(&mut ps);
                // self.pendulums.ps.extend(next);
                // self.state = GameState::RUN;
            }
        }
    }
}
