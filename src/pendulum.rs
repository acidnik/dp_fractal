use std::collections::VecDeque;
use std::f32::consts::PI;
use std::time::Duration;

use ggez::graphics::{self, Color, DrawMode, FillOptions, Rect};
use ggez::*;
use glam::*;

const G: f32 = 9.81;
const PHI: f32 = 2.0 * PI;
const MAX_STEP: usize = 128_000; //63020;

#[derive(Clone, Debug)]
pub struct DoublePendulum {
    pub p:      Vec2, // fixed point
    pub theta1: f32,  // angle of first arm: 0 .. PHI
    pub theta2: f32,  // angle of snd arm: 0 .. PHI
    pub l1:     f32,  // len of first arm
    pub l2:     f32,  // len of snd arm
    pub dt1:    f32,  //
    pub dt2:    f32,  //
    pub d2t1:   f32,
    pub d2t2:   f32,
    scale: f32,

    pub stopped: bool,
    steps:       usize,
    pub hist:    VecDeque<f32>,
}

impl DoublePendulum {
    pub fn new(p: Vec2, theta1: f32, theta2: f32, scale: f32) -> Self {
        DoublePendulum {
            p:       p,
            theta1:  theta1,
            theta2:  theta2,
            l1:      80.0,
            l2:      80.0,
            dt1:     0.0,
            dt2:     0.0,
            d2t1:    0.0,
            d2t2:    0.0,
            scale: scale,
            stopped: false,
            steps:   0,
            hist:    VecDeque::new(),
        }
    }
    
    pub fn new2(p: Vec2, width: f32, scale: f32) -> Self {
        // p.x .. w => 0 .. PHI
        // p.y .. w => 0 .. PI
        let theta1 = p.x / width * PHI;
        let theta2 = p.y / width * PI;
        let mut this = DoublePendulum::new(p, theta1, theta2, scale);
        this.l1 = width / 4.0;
        this.l2 = width / 4.0;
        this
    }
    
    pub fn split(&mut self, width: f32) -> Vec<DoublePendulum> {
        assert!(self.stopped);
        if self.l1 < 8.0 {
            return vec![];
        }
        let mut res = Vec::new();
        let d = self.l1 * self.scale;
        if self.p.x - self.l1 > 0.0 && self.p.y - self.l1 > 0.0 {
            res.push(
                DoublePendulum::new2(vec2(self.p.x-self.l1, self.p.y-self.l1), width, self.scale/2.0),
            )
        }
        if self.p.x - self.l1 > 0.0 && self.p.y + self.l1 < width {
            res.push(
                DoublePendulum::new2(vec2(self.p.x-self.l1, self.p.y+self.l1), width, self.scale/2.0),
            )
        }
        if self.p.x + self.l1 < width && self.p.y - self.l1 > 0.0 {
            res.push(
                DoublePendulum::new2(vec2(self.p.x+self.l1, self.p.y-self.l1), width, self.scale/2.0),
            )
        }
        if self.p.x + self.l1 < width && self.p.y + self.l1 < width {
            res.push(
                DoublePendulum::new2(vec2(self.p.x+self.l1, self.p.y+self.l1), width, self.scale/2.0),
            )
        }

        res
    }

    pub fn update(&mut self, ctx: &mut Context) -> GameResult<()> {
        let delta = timer::delta(ctx).as_secs_f32();
        // let steps = 120;
        let steps = 400;
        // let steps = 20;
        for _ in 1..=steps {
            self.step(0.001);
        }

        Ok(())
    }

    fn step(&mut self, delta: f32) {
        if self.stopped {
            return;
        }
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

        self.dt1 += self.d2t1 * delta;
        self.dt2 += self.d2t2 * delta;

        self.theta1 += self.dt1 * delta;
        self.theta2 += self.dt2 * delta;

        // if self.theta1 > PHI {
        //     self.theta1 -= PHI
        // }
        // else if self.theta1 < -PHI {
        //     self.theta1 += PHI
        // }

        // if self.theta2 > PHI {
        //     self.theta2 -= PHI
        // }
        // else if self.theta2 < -PHI {
        //     self.theta2 += PHI
        // }

        // dt2 > 0 -> clockwise; dt2 < 0 - counter-cw
        // -3.13 -> -3.15;  dt-
        // 3.15 -> 3.13     dt-
        // -3.15 -> -3.13;  dt+
        // 3.13 -> 3.15;    dt+

        if self.hist.len() > 0 {
            let prev = self.hist[self.hist.len() - 1];
            if (self.dt2 > 0.0 && ((self.theta2 < -PI && prev > -PI) || (self.theta2 > PI && prev < PI)))
                || (self.dt2 < 0.0 && ((self.theta2 < -PI && prev > -PI) || (self.theta2 < PI && prev > PI)))
            {
                self.stopped = true;
                self.hist.clear();
                println!("{:.0}", self.steps);
            }
        }

        self.hist.push_back(self.theta2);
        if self.hist.len() > 2 {
            self.hist.pop_front();
        }
        // if self.hist.len() % 700 == 0 {
        //     println!("{:.6} {:.6}", self.theta2, self.dt2)
        // }
    }

    pub fn draw(&self, ctx: &mut Context) -> GameResult<()> {
        let mb = &mut graphics::MeshBuilder::new();
        let tint = self.steps as f32 / MAX_STEP as f32;
        let mut rtint = self.theta1;
        if rtint > PHI {
            rtint -= PHI
        }
        else if rtint < -PHI {
            rtint += PHI
        }
        rtint /= PHI;
        let color = [tint, tint, rtint, 1.0].into();
        if self.stopped {
            let w = self.scale * (self.l1 + self.l2);
            // let tint = 0.5;
            // println!("{:.6}", tint);
            let draw_mode = DrawMode::fill();
            mb.rectangle(draw_mode, Rect::new(self.p.x - w, self.p.y - w, 2.0 * w, 2.0 * w), color)?;
        }
        else {
            let mut p1 = self.p.clone();
            let (ts1, tc1) = self.theta1.sin_cos();
            p1.x += self.scale * ts1 * self.l1;
            p1.y += self.scale * tc1 * self.l1;

            let mut p2 = p1.clone();
            let (ts2, tc2) = self.theta2.sin_cos();
            p2.x += self.scale * ts2 * self.l2;
            p2.y += self.scale * tc2 * self.l2;

            mb.line(&[self.p, p1, p2], 3.0, color)?;
        }
        let mesh = mb.build(ctx)?;
        graphics::draw(ctx, &mesh, (vec2(0.0, 0.0), Color::WHITE))?;
        Ok(())
    }
}
