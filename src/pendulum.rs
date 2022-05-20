use std::cell::RefCell;
use std::collections::{HashMap, VecDeque};
use std::f32::consts::PI;
use std::rc::Rc;
use std::sync::Mutex;
use std::time::Duration;

use ggez::graphics::{self, Color, DrawMode, FillOptions, Rect};
use ggez::*;
use glam::*;

const G: f32 = 9.81;
const PHI: f32 = 2.0 * PI;
const MAX_STEP: usize = 300_000;
const MIN_PIXEL: f32 = 16.0;
const UPDATE_STEPS: usize = 120;
const STEP_DELTA: f32 = 0.001;

#[derive(Clone, Debug)]
pub struct DoublePendulum {
    id:     usize,
    childs: Vec<usize>,
    parent_id: usize,

    pub p:      Vec2, // fixed point
    pub theta1: f32,  // angle of first arm: 0 .. PHI
    pub theta2: f32,  // angle of snd arm: 0 .. PHI
    pub l1:     f32,  // len of first arm
    pub l2:     f32,  // len of snd arm
    pub dt1:    f32,  //
    pub dt2:    f32,  //
    pub d2t1:   f32,
    pub d2t2:   f32,
    scale:      f32,

    pub stopped: bool,
    pub steps:   usize,
    pub prev:    f32,
}

impl DoublePendulum {
    pub fn new(p: Vec2, theta1: f32, theta2: f32, scale: f32) -> Self {
        let this = DoublePendulum {
            id:      0,
            parent_id: 0,
            childs:  Vec::new(),
            p:       p,
            theta1:  theta1,
            theta2:  theta2,
            l1:      80.0,
            l2:      80.0,
            dt1:     0.0,
            dt2:     0.0,
            d2t1:    0.0,
            d2t2:    0.0,
            scale:   scale,
            stopped: false,
            steps:   0,
            prev:    f32::INFINITY,
        };
        this
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
        if ((self.l1 * self.scale) - MIN_PIXEL).abs() < 0.01 {
            return vec![];
        }
        let mut res = Vec::new();
        let d = self.l1 * self.scale;
        if self.p.x - d > 0.0 && self.p.y - d > 0.0 {
            res.push(DoublePendulum::new2(vec2(self.p.x - d, self.p.y - d), width, self.scale / 2.0))
        }
        if self.p.x - d > 0.0 && self.p.y + d < width {
            res.push(DoublePendulum::new2(vec2(self.p.x - d, self.p.y + d), width, self.scale / 2.0))
        }
        if self.p.x + d < width && self.p.y - d > 0.0 {
            res.push(DoublePendulum::new2(vec2(self.p.x + d, self.p.y - d), width, self.scale / 2.0))
        }
        if self.p.x + d < width && self.p.y + d < width {
            res.push(DoublePendulum::new2(vec2(self.p.x + d, self.p.y + d), width, self.scale / 2.0))
        }

        for r in &res {
            self.childs.push(r.id);
        }

        res
    }

    pub fn update(&mut self, ctx: &mut Context) -> GameResult<()> {
        let delta = timer::delta(ctx).as_secs_f32();
        for _ in 1..=UPDATE_STEPS {
            self.step(STEP_DELTA);
        }

        Ok(())
    }

    fn step(&mut self, delta: f32) {
        if self.stopped {
            return;
        }
        if self.steps >= MAX_STEP {
            self.stopped = true;
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

        if self.prev.is_finite() {
            if (self.dt2 > 0.0 && ((self.theta2 < -PI && self.prev > -PI) || (self.theta2 > PI && self.prev < PI)))
                || (self.dt2 < 0.0 && ((self.theta2 < -PI && self.prev > -PI) || (self.theta2 < PI && self.prev > PI)))
            {
                self.stopped = true;
                // println!("stop = {}", self.steps);
            }
        }

        self.prev = self.theta2;
    }

    pub fn draw(&self, ctx: &mut Context) -> GameResult<()> {
        // if self.l1 * self.scale < 1.0 {
        //     return Ok(())
        // }
        let mb = &mut graphics::MeshBuilder::new();
        // let tint = ((self.steps as f32).ln() + 1.0) / (MAX_STEP as f32).ln() * 3.0;
        let tint = ((self.steps as f32).ln().sin() + 1.0) / 2.0 * 3.0;
        // let tint = self.steps as f32 / MAX_STEP as f32 * 3.0;
        let (r, g, b) = if tint > 2.0 {
            (1.0, 1.0, tint - 2.0)
        }
        else if tint > 1.0 {
            (1.0, tint - 1.0, 0.0)
        }
        else {
            (tint, 0.0, 0.0)
        };
        let color = [r, g, b, 1.0].into();
        let pcolor = [1.0 - r, 1.0 - g, 1.0 - b, 1.0].into();
        // ................
        // let tint = 0.33 + (self.steps as f32 / MAX_STEP as f32) * 0.666;
        // let mut rtint = self.theta1;
        // if rtint > PHI {
        //     rtint -= PHI
        // }
        // else if rtint < -PHI {
        //     rtint += PHI
        // }
        // rtint = 0.95 + (rtint / PHI * 0.05);
        // let color = [tint, tint, rtint, 1.0].into();
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

            mb.line(&[self.p, p1, p2], 3.0, pcolor)?;
        }
        let mesh = mb.build(ctx)?;
        graphics::draw(ctx, &mesh, (vec2(0.0, 0.0), Color::WHITE))?;
        Ok(())
    }
            
    pub fn point_inside(&self, x: f32, y: f32) -> bool {
        let w = (self.l1 + self.l2) * self.scale;
        x > self.p.x - w && x < self.p.x + w && y > self.p.y - w && y < self.p.y + w
    }
        
    pub fn area(&self) -> f32 {
        let w = (self.l1 + self.l2) * self.scale;
        w*w
    }
}

pub struct PendulumFamily {
    pub ps:   HashMap<usize, Rc<RefCell<DoublePendulum>>>,
    pub done: HashMap<usize, Rc<RefCell<DoublePendulum>>>,
    counter:  usize,
}

impl PendulumFamily {
    pub fn new() -> Self {
        PendulumFamily {
            ps:      HashMap::new(),
            done:    HashMap::new(),
            counter: 0,
        }
    }

    fn next_id(&mut self) -> usize {
        self.counter += 1;
        self.counter
    }

    pub fn add(&mut self, mut p: DoublePendulum) {
        p.id = self.next_id();
        self.ps.insert(p.id, Rc::new(RefCell::new(p)));
    }
    
    pub fn can_remove(&self, id: usize) -> bool {
        let p = self.done.get(&id);
        if p.is_none() {
            return false
        }
        let p = p.unwrap().borrow();
        for c in &p.childs {
            if let Some(c) = self.done.get(&c) {
                if ! c.borrow().stopped {
                    return false
                }
            }
        }
        true
    }
    
    pub fn len(&self) -> usize {
        self.ps.len()
    }
    
    pub fn find(&self, x: f32, y: f32) -> Option<Rc<RefCell<DoublePendulum>>> {
        let mut result = None;
        for pref in self.ps.values() {
            let p = pref.borrow();
            if p.point_inside(x, y) {
                if result.is_none() {
                    result = Some(pref.clone())
                }
                else if let Some(ref r) = result {
                    if r.borrow().area() > p.area() {
                        result = Some(pref.clone())
                    }
                }
            }
        }
        result
    }

    pub fn update(&mut self, ctx: &mut Context, width: f32) -> GameResult<()> {
        let mut next = HashMap::new();
        let mut stopped = Vec::new();
        for pref in &mut self.ps.values_mut() {
            let mut p = pref.borrow_mut();
            p.update(ctx)?;
            if p.stopped {
                self.done.insert(p.id, pref.clone());
                for mut n in p.split(width) {
                    self.counter += 1;
                    n.id = self.counter;
                    n.parent_id = p.id;
                    next.insert(n.id, Rc::new(RefCell::new(n)));
                }
                stopped.push(p.parent_id);
            }
            else {
                next.insert(p.id, pref.clone());
            }
        }
        for pid in stopped {
            if self.can_remove(pid) {
                self.done.remove(&pid);
            }
        }
        if self.ps.len() > 0 && self.ps.len() % 100 == 0 {
            println!("active: {}, done: {}", self.ps.len(), self.done.len());
        }
        self.ps = next;
        Ok(())
    }
    
    pub fn draw(&self, ctx: &mut Context) -> GameResult<()> {
        for p in self.done.values() {
            p.borrow().draw(ctx)?
        }
        for p in self.ps.values() {
            p.borrow().draw(ctx)?
        }
        Ok(())
    }
}
