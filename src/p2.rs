use std::cell::RefCell;
use std::collections::{HashMap, HashSet, VecDeque};
use std::f64::consts::{PI, TAU};
use std::rc::Rc;
use std::sync::mpsc::channel;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use angular_units::{Angle, Deg, Rad};
use crossbeam::channel::bounded;
use ggez::graphics::{self, get_window_color_format, Canvas, Color, DrawMode, FillOptions, Rect};
use ggez::*;
use glam::*;
use prisma::Lerp;

use crate::avgspeed::RollingAverage;

const G: f64 = 9.81;
const L1: f64 = 80.0;

// dt for physics
const STEP_DELTA: f64 = 0.01;

#[derive(Clone)]
pub struct Config {
    // the min angle of first arm
    pub xmin:         f64,
    // misnomer. this is the with of the area in terms of width
    pub xmax:         f64,
    pub ymin:         f64,
    pub ymax:         f64,
    pub min_pixel:    f64,
    pub update_steps: usize,
}

#[derive(Clone, Debug)]
pub struct DoublePendulum {
    id:         usize,
    pub p:      DVec2, // fixed point
    pub theta1: f64,   // angle of first arm: 0 .. TAU
    pub theta2: f64,   // angle of snd arm: 0 .. TAU
    pub l1:     f64,   // len of first arm
    pub l2:     f64,   // len of snd arm
    pub dt1:    f64,   //
    pub dt2:    f64,   //
    pub d2t1:   f64,
    pub d2t2:   f64,
    pub scale:  f64,

    pub steps: usize,
    color:     Color,
}

impl DoublePendulum {
    pub fn new(p: DVec2, theta1: f64, theta2: f64, scale: f64) -> Self {
        let this = DoublePendulum {
            id:     0,
            p:      p,
            theta1: theta1,
            theta2: theta2,
            l1:     L1,
            l2:     L1,
            dt1:    0.0,
            dt2:    0.0,
            d2t1:   0.0,
            d2t2:   0.0,
            scale:  scale,
            steps:  0,
            color:  Color::WHITE,
        };
        this
    }

    pub fn new2(p: DVec2, width: f64, scale: f64, config: &Config) -> Self {
        // p.x .. w => 0 .. TAU
        // p.y .. w => 0 .. PI
        //let theta1 = config.xmin + p.x / width * config.xmax;
        //let theta2 = config.ymin + p.y / width * config.ymax;
        let theta1 = PI;
        let theta2 = PI / 2.0;
        let mut this = DoublePendulum::new(p, theta1, theta2, scale);
        this.l1 = 10.0 * ( p.x / width + 1.0);
        this.l2 = 10.0 * ( p.y / width + 1.0);
        this
    }

    pub fn update(&mut self, update_steps: usize) -> GameResult<()> {
        for _ in 1..=update_steps {
            self.step(STEP_DELTA);
        }

        Ok(())
    }

    fn step(&mut self, delta: f64) {
        self.steps += 1;

        let a = 2.0 * self.l1 + self.l2 - self.l2 * (2.0 * self.theta1 - 2.0 * self.theta2).cos();

        self.d2t1 = (-G * (2.0 * self.l1 + self.l2) * self.theta1.sin()
            - self.l2 * G * (self.theta1 - 2.0 * self.theta2).sin()
            - 2.0
                * (self.theta1 - self.theta2).sin()
                * self.l2
                * (self.dt2 * self.dt2 * self.l2 - self.dt1 * self.dt1 * self.l1 * (self.theta1 - self.theta2).cos()))
            / (self.l1 * a);

        self.d2t2 = (2.0
            * (self.theta1 - self.theta2).sin()
            * (self.dt1 * self.dt1 * self.l1 * (self.l1 + self.l2)
                + G * (self.l1 + self.l2) * self.theta1.cos()
                + self.dt2 * self.dt2 * self.l2 * self.l2 * (self.theta1 - self.theta2).cos()))
            / (self.l2 * a);

        if self.d2t1.is_nan()
            || self.dt1.is_nan()
            || self.theta1.is_nan()
            || self.d2t2.is_nan()
            || self.dt2.is_nan()
            || self.theta2.is_nan()
        {
            println!(
                "{} {} {} {} {} {} {} {}",
                self.l1, self.l2, self.theta1, self.dt1, self.d2t1, self.theta2, self.dt2, self.d2t2
            );
            panic!("");
        }

        self.dt1 += self.d2t1 * delta;
        self.dt2 += self.d2t2 * delta;

        self.theta1 += self.dt1 * delta;
        self.theta2 += self.dt2 * delta;

        // while self.theta1 > TAU {
        //     self.theta1 -= TAU
        // }
        // while self.theta1 < -TAU {
        //     self.theta1 += TAU
        // }
        // while self.theta2 > TAU {
        //     self.theta2 -= TAU
        // }
        // while self.theta2 < -TAU {
        //     self.theta2 += TAU
        // }
    }

    fn color(&mut self) -> Color {
        // self.color
        // let mut p = (self.theta1 + self.theta2).abs();
        let mut p = self.theta1.abs();
        while p >= TAU {
            p -= TAU
        }
        let cr: prisma::Rgb<f32> = prisma::Hsv::new(Rad::new(p), 0.7, 1.0).into();
        (cr.red(), cr.green(), cr.blue()).into()
        // (p as f32/ TAU as f32, p as f32 / TAU as f32, p as f32 / TAU as f32).into()
    }

    #[inline(always)]
    fn rect(&self, x1: f64, y1: f64, w: f64, h: f64) -> Rect {
        Rect::new(x1 as f32, y1 as f32, w as f32, h as f32)
    }

    pub fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
        let w = self.scale * 2048.0;
        let draw_mode = DrawMode::fill();
        let color = self.color();
        let mb = &mut graphics::MeshBuilder::new();

        mb.rectangle(draw_mode, self.rect(self.p.x - w, self.p.y - w, 2.0 * w, 2.0 * w), color)?;

        let mesh = mb.build(ctx)?;
        graphics::draw(ctx, &mesh, (vec2(0.0, 0.0), Color::WHITE))?;
        Ok(())
    }

    fn width(&self) -> f64 {
        (self.l1 + self.l2) * self.scale
    }
}

pub struct PendulumFamily2 {
    config:           Config,
    width:            f64,
    pub ps:           Vec<DoublePendulum>,
    pub iter:         usize,
    pub update_steps: usize,
    t:                Instant,
}

impl PendulumFamily2 {
    pub fn new(ctx: &mut Context, config: Config, width: f64) -> Self {
        Self {
            config:       config,
            width:        width,
            ps:           Vec::new(),
            iter:         0,
            update_steps: 100,
            t:            Instant::now(),
        }
    }

    pub fn init(&mut self) {
        let cnt = (self.width / self.config.min_pixel) as usize;
        for i in 0..cnt {
            for j in 0..cnt {
                self.ps.push(DoublePendulum::new2(
                    dvec2(i as f64 * self.config.min_pixel, j as f64 * self.config.min_pixel),
                    self.width,
                    self.config.min_pixel / self.width,
                    &self.config,
                ));
            }
        }
        println!("init done: {}", self.ps.len());
    }

    pub fn len(&self) -> usize {
        self.ps.len()
    }

    pub fn update(&mut self, ctx: &mut Context) -> GameResult<()> {
        self.iter += 1;
        let update_steps = self.config.update_steps;
        for p in &mut self.ps {
            p.update(update_steps)?;
        }
        println!("{} {:?}", self.iter, self.t.elapsed());
        self.t = Instant::now();
        Ok(())
    }

    pub fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
        let t = Instant::now();
        for p in &mut self.ps {
            p.draw(ctx)?
        }
        println!("draw: {:?}", t.elapsed());
        Ok(())
    }
}
