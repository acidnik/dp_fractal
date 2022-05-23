use std::cell::RefCell;
use std::collections::{HashMap, HashSet, VecDeque};
use std::f32::consts::PI;
use std::rc::Rc;
use std::sync::Mutex;
use std::time::Duration;

use angular_units::{Angle, Deg, Rad};
use ggez::graphics::{self, Color, DrawMode, FillOptions, Rect};
use ggez::*;
use glam::*;
use kiddo::distance::squared_euclidean;
use kiddo::KdTree;
use prisma::Lerp;

const G: f32 = 9.81;
const PHI: f32 = 2.0 * PI;
const L1: f32 = 80.0;
const MIN_PIXEL: f32 = 32.0;
const MIN_DIVE_PIXEL: f32 = 8.0;
const UPDATE_STEPS: usize = 120;
// const UPDATE_STEPS: usize = 10;
const STEP_DELTA: f32 = 0.01;
const UPDATE_STEP: f32 = UPDATE_STEPS as f32 * STEP_DELTA;
const MAX_STEP: usize = (UPDATE_STEP * 1_000_000.0) as usize;
const COLOR_STEP: usize = (20.0 / UPDATE_STEP) as usize;
const DIVE_DIFF: f32 = 0.9;

#[derive(Clone, Debug)]
pub struct DoublePendulum {
    id:        usize,
    childs:    Vec<usize>,
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
            id:        0,
            parent_id: 0,
            childs:    Vec::new(),
            p:         p,
            theta1:    theta1,
            theta2:    theta2,
            l1:        L1,
            l2:        L1,
            dt1:       0.0,
            dt2:       0.0,
            d2t1:      0.0,
            d2t2:      0.0,
            scale:     scale,
            stopped:   false,
            steps:     0,
            prev:      f32::INFINITY,
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

    pub fn split(&mut self, width: f32, min_pixel: f32) -> Vec<DoublePendulum> {
        assert!(self.stopped);
        if self.width() < min_pixel {
            return vec![];
        }
        let mut res = Vec::new();
        let d = self.width() / 2.0;
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

    fn color(&self) -> (f32, f32, f32) {
        if self.steps == MAX_STEP {
            let cr = prisma::Rgb::new(1.0, 1.0, 1.0);
            return (cr.red(), cr.green(), cr.blue());
        }
        // let p = ((self.steps / 100 % 360) as f32) / 1_000.0 * PHI;
        let p = ((self.steps / COLOR_STEP % 360) as f32 / 360.0) * PHI;
        // println!("{} {:.6}", self.steps, p);
        let c = prisma::Hsv::new(Rad::new(p), 1.0, 1.0);
        // let p = 0.9;
        // let clow = prisma::Hsv::new(Rad::new(0.0), 1.0, 1.0);
        // let chigh = prisma::Hsv::new(Rad::new(PHI), 1.0, 1.0);
        // let c = clow.lerp(&chigh, p);
        let cr: prisma::Rgb<f32> = c.into();
        // println!("{:?} => {:?} => {:?}", c, p, cr);
        (cr.red(), cr.green(), cr.blue())
    }

    pub fn draw(&self, ctx: &mut Context) -> GameResult<()> {
        // if self.l1 * self.scale < 1.0 {
        //     return Ok(());
        // }
        let mb = &mut graphics::MeshBuilder::new();
        let color = self.color();
        let (r, g, b) = color;
        let color = color.into();
        let pcolor = [1.0 - r, 1.0 - g, 1.0 - b, 1.0].into();
        if self.stopped {
            let w = self.scale * (self.l1 + self.l2);
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

    fn width(&self) -> f32 {
        (self.l1 + self.l2) * self.scale
    }

    fn adjacent(&self, other: &DoublePendulum) -> bool {
        let epsilon = MIN_DIVE_PIXEL / 4.0;
        let w2 = self.width() + other.width();
        let dx = (self.p.x - other.p.x).abs();
        let dy = (self.p.y - other.p.y).abs();
        (dx <= w2 && (dy - w2).abs() < epsilon) || (dy <= w2 && (dx - w2).abs() < epsilon)
    }

    pub fn area(&self) -> f32 {
        let w = (self.l1 + self.l2) * self.scale;
        w * w
    }
}

pub struct PendulumFamily {
    width:    f32,
    pub ps:   HashMap<usize, Rc<RefCell<DoublePendulum>>>,
    queue:    VecDeque<Rc<RefCell<DoublePendulum>>>,
    pub done: HashMap<usize, Rc<RefCell<DoublePendulum>>>,
    tree:     KdTree<f32, usize, 2>,
    counter:  usize,
    iter:     usize,
}

impl PendulumFamily {
    pub fn new(width: f32) -> Self {
        println!(
            "steps_delta = {:.6}; stop after {}; dive if {}; p = {}; color step = {}",
            UPDATE_STEP, MAX_STEP, DIVE_DIFF, MIN_DIVE_PIXEL, COLOR_STEP
        );
        PendulumFamily {
            width:   width,
            ps:      HashMap::new(),
            queue:   VecDeque::new(),
            done:    HashMap::new(),
            tree:    KdTree::new(),
            counter: 0,
            iter:    0,
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
            return false;
        }
        let p = p.unwrap().borrow();
        for c in &p.childs {
            if let Some(c) = self.done.get(&c) {
                if !c.borrow().stopped {
                    return false;
                }
            }
        }
        true
    }

    pub fn len(&self) -> usize {
        self.ps.len()
    }

    pub fn find(&self, x: f32, y: f32) -> Option<Rc<RefCell<DoublePendulum>>> {
        let node = self.tree.nearest(&[x, y], 1, &squared_euclidean).ok()?;
        if node.len() == 0 {
            return None;
        }
        let res = self.done.get(node[0].1);
        if res.is_none() {
            return None;
        }
        if let Some(res) = res {
            return Some(res.clone());
        }
        None
    }

    pub fn dive(&self, p: &mut DoublePendulum) -> Vec<DoublePendulum> {
        /* if adjacent stopped pendulums has large diff with p - split p and adjacent into smaller pixels
         */
        assert!(p.stopped);
        if p.width() < MIN_DIVE_PIXEL {
            return vec![];
        }
        let nearest = self.tree.nearest(&[p.p.x, p.p.y], 10, &squared_euclidean).unwrap();
        let mut res = Vec::new();
        let mut skip = HashSet::new();
        skip.insert(p.id);
        for id in nearest {
            if skip.contains(id.1) {
                continue;
            }
            if let Some(mut n) = self.done.get(id.1) {
                let mut n = n.borrow_mut();
                if !n.stopped || n.width() < MIN_DIVE_PIXEL {
                    continue;
                }
                let (asteps, bsteps) = (p.steps.min(n.steps), p.steps.max(n.steps));
                let psteps = asteps as f32 / bsteps as f32;
                if psteps < DIVE_DIFF && p.adjacent(&n) {
                    // println!("{} / {} = {:.3}", asteps, bsteps, psteps);
                    res.extend(n.split(self.width, MIN_DIVE_PIXEL));
                    skip.insert(n.id);
                    println!("split {} {}", self.iter, n.id);
                    println!("split {}", n.id);
                }
            }
        }
        if res.len() > 0 {
            res.extend(p.split(self.width, MIN_DIVE_PIXEL));
            // println!("split {}", p.id);
        }
        res
    }

    pub fn update(&mut self, ctx: &mut Context) -> GameResult<()> {
        self.iter += 1;
        let mut next = HashMap::new();
        let mut stopped = Vec::new();
        for pref in &mut self.ps.values_mut() {
            let mut p = pref.borrow_mut();
            p.update(ctx)?;
            if p.stopped {
                self.done.insert(p.id, pref.clone());
                self.tree.add(&p.p.to_array(), p.id).unwrap();
                // for mut n in p.split(self.width, MIN_PIXEL) {
                //     self.counter += 1;
                //     n.id = self.counter;
                //     n.parent_id = p.id;
                //     next.insert(n.id, Rc::new(RefCell::new(n)));
                // }
                stopped.push(pref.clone());
            }
            else {
                next.insert(p.id, pref.clone());
            }
        }
        for p in &stopped {
            let p = p.borrow();
            if self.can_remove(p.parent_id) {
                self.done.remove(&p.parent_id);
                self.tree.remove(&p.p.to_array(), &p.id).unwrap();
            }
        }
        // for p in &mut stopped {
        //     let mut p = p.borrow_mut();
        //     for mut d in self.dive(&mut p) {
        //         self.counter += 1;
        //         d.id = self.counter;
        //         d.parent_id = p.id;
        //         if next.len() > 500000 {
        //             self.queue.push_back(Rc::new(RefCell::new(d)))
        //         }
        //         else {
        //             next.insert(d.id, Rc::new(RefCell::new(d)));
        //         }
        //     }
        // }
        while next.len() < 500 && self.queue.len() > 0 {
            let n = self.queue.pop_front().unwrap();
            next.insert(n.borrow().id, n.clone());
        }
        if self.ps.len() > 0 && (self.ps.len() + self.queue.len()) % 100 == 0 {
            println!("active: {}, queued: {}, done: {}", self.ps.len(), self.queue.len(), self.done.len());
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


mod test {
    use glam::vec2;

    use super::{DoublePendulum, L1};

    #[test]
    fn test_adjacent() {
        let WIDTH = 2048.0;
        let p1 = DoublePendulum::new2(vec2(WIDTH / 4.0, WIDTH / 4.0), WIDTH, 0.5);
        let p2 = DoublePendulum::new2(vec2(WIDTH / 4.0, WIDTH / 4.0 * 3.0), WIDTH, 0.5);
        assert!(p1.adjacent(&p2));
        let mut p3 = p2.clone();
        p3.p.y += 1.0;
        assert!(!p1.adjacent(&p3));
    }

}