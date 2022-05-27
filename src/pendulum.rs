use std::cell::RefCell;
use std::collections::{HashMap, HashSet, VecDeque};
use std::f32::consts::{PI, TAU};
use std::rc::Rc;
use std::sync::{Mutex, Arc};
use std::sync::mpsc::channel;
use std::thread;
use std::time::{Duration, Instant};

use angular_units::{Angle, Deg, Rad};
use crossbeam::channel::bounded;
use ggez::graphics::{self, get_window_color_format, Canvas, Color, DrawMode, FillOptions, Rect};
use ggez::*;
use glam::*;
use kiddo::distance::squared_euclidean;
use kiddo::KdTree;
use prisma::Lerp;

use crate::avgspeed::RollingAverage;

const G: f32 = 9.81;
const L1: f32 = 80.0;
const MIN_PIXEL: f32 = 4.0;
const MIN_DIVE_PIXEL: f32 = 8.0;
const STEP_DELTA: f32 = 0.01;
const COLOR_STEP: f32 = 15.0;
const DIVE_DIFF: f32 = 0.92;

#[derive(Clone, Debug)]
pub struct DoublePendulum {
    pub id:    usize,
    childs:    Vec<usize>,
    parent_id: usize,

    pub p:      Vec2, // fixed point
    pub theta1: f32,  // angle of first arm: 0 .. TAU
    pub theta2: f32,  // angle of snd arm: 0 .. TAU
    pub l1:     f32,  // len of first arm
    pub l2:     f32,  // len of snd arm
    pub dt1:    f32,  //
    pub dt2:    f32,  //
    pub d2t1:   f32,
    pub d2t2:   f32,
    pub scale:  f32,

    pub stopped: bool,
    pub steps:   usize,
    pub prev:    f32,
    color:       Color,
    expired:     bool,
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
            color:     Color::WHITE,
            expired:   false,
        };
        this
    }

    pub fn new2(p: Vec2, width: f32, scale: f32) -> Self {
        // p.x .. w => 0 .. TAU
        // p.y .. w => 0 .. PI
        //let theta1 = p.x / width * TAU;
        //let theta2 = p.y / width * PI;
        // (480, 1056)
        let theta1 = 1.46 + p.x / width * 0.1;
        let theta2 = 1.61 + p.y / width * 0.1;
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

        for r in &mut res {
            self.childs.push(r.id);
            r.color = self.color;
        }

        res
    }

    pub fn update(&mut self, ctx: &mut Context, update_steps: usize, max_step: usize) -> GameResult<()> {
        for _ in 1..=update_steps {
            self.step(STEP_DELTA, max_step);
        }

        Ok(())
    }

    fn step(&mut self, delta: f32, max_step: usize) {
        if self.stopped {
            return;
        }
        if self.steps >= 8000 + (max_step as f32 / self.scale) as usize {
            self.stopped = true;
            self.expired = true;
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

    fn update_color(&mut self, color_step: usize) {
        if self.expired {
            let cr = prisma::Rgb::new(0.7, 0.7, 0.7);
            self.color = (cr.red(), cr.green(), cr.blue()).into();
        }
        else {
            let p = (self.steps / color_step % 1_000) as f32 / 1_000.0 * TAU;
            let c = prisma::Hsv::new(Rad::new(p), 1.0, 1.0);
            let cr: prisma::Rgb<f32> = c.into();
            self.color = (cr.red(), cr.green(), cr.blue()).into();
        }
    }

    fn color(&mut self) -> Color {
        self.color
    }

    fn pcolor(&self) -> Color {
        let mut p = self.theta2.abs();
        while p > TAU {
            p -= TAU
        }
        let cr: prisma::Rgb<f32> = prisma::Hsv::new(Rad::new(p), 0.5, 1.0).into();
        (cr.red(), cr.green(), cr.blue()).into()
    }

    pub fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
        // if self.l1 * self.scale < 1.0 {
        //     return Ok(());
        // }
        let w = self.scale * (self.l1 + self.l2);
        let draw_mode = DrawMode::fill();
        let color = self.color();
        let mb = &mut graphics::MeshBuilder::new();
        if self.stopped {
            mb.rectangle(draw_mode, Rect::new(self.p.x - w, self.p.y - w, 2.0 * w, 2.0 * w), color)?;
        }
        else {
            let pcolor = self.pcolor();
            let mut p1 = self.p.clone();
            let (ts1, tc1) = self.theta1.sin_cos();
            p1.x += self.scale * ts1 * self.l1;
            p1.y += self.scale * tc1 * self.l1;

            let mut p2 = p1.clone();
            let (ts2, tc2) = self.theta2.sin_cos();
            p2.x += self.scale * ts2 * self.l2;
            p2.y += self.scale * tc2 * self.l2;

            mb.rectangle(draw_mode, Rect::new(self.p.x - w, self.p.y - w, 2.0 * w, 2.0 * w), color)?;
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
        let epsilon = 0.1;
        let w2 = self.width() + other.width();
        let dx = (self.p.x - other.p.x).abs();
        let dy = (self.p.y - other.p.y).abs();
        // println!("{} {} {}", dx, dy, w2);
        (dx <= w2 && (dy - w2).abs() < epsilon) || (dy <= w2 && (dx - w2).abs() < epsilon)
    }

}

pub struct PendulumFamily {
    width:    f32,
    pub ps:   HashMap<usize, Rc<RefCell<DoublePendulum>>>,
    pub done: HashMap<usize, Rc<RefCell<DoublePendulum>>>,
    tree:     KdTree<f32, usize, 2>,
    pub dive: HashSet<usize>,
    to_draw:  VecDeque<Rc<RefCell<DoublePendulum>>>,
    counter:  usize,
    pub iter: usize,
    canvas:   Canvas,
    avg:      RollingAverage<u32>,

    pub update_steps: usize,
    max_steps:        usize,
}

impl PendulumFamily {
    pub fn new(ctx: &mut Context, width: f32) -> Self {
        PendulumFamily {
            width:   width,
            ps:      HashMap::new(),
            done:    HashMap::new(),
            tree:    KdTree::new(),
            counter: 0,
            iter:    0,
            dive:    HashSet::new(),
            to_draw: VecDeque::new(),
            canvas:  Canvas::with_window_size(ctx).unwrap(),
            avg:     RollingAverage::new(1000),

            update_steps: 100,
            max_steps:    2000,
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

    #[allow(dead_code)]
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

    pub fn find_all(&self, x: f32, y: f32) -> Option<Rc<RefCell<DoublePendulum>>> {
        let mut res = Vec::new();
        for (_, pref) in self.done.iter().chain(self.ps.iter()) {
            let p = pref.borrow();
            if p.point_inside(x, y) {
                res.push(pref.clone())
            }
            // if p.borrow_mut().point_inside(x, y) {
            //     return Some(p.clone());
            // }
        }
        res.sort_by(|a, b| {
            (a.borrow().scale, a.borrow().stopped)
                .partial_cmp(&(b.borrow().scale, b.borrow().stopped))
                .unwrap()
        });
        match res.len() {
            0 => None,
            1 => Some(res[0].clone()),
            2 => {
                println!("found {:?}", res);
                Some(res[0].clone())
            }
            _ => {
                println!("found {:?}", res.iter().map(|p| p.borrow().stopped).collect::<Vec<_>>());
                Some(res[0].clone())
            }
        }
    }

    pub fn dive(&mut self, p: &mut DoublePendulum) -> Vec<DoublePendulum> {
        /* if adjacent stopped pendulums has large diff with p - split p and adjacent into smaller pixels
         */
        assert!(p.stopped);
        // if p.width() < MIN_DIVE_PIXEL {
        //     return vec![];
        // }
        let nearest = self.tree.nearest(&[p.p.x, p.p.y], 100, &squared_euclidean).unwrap();
        let mut res = Vec::new();
        let mut skip = HashSet::new();
        skip.insert(p.id);
        // if neighbor is different, but already in split list - split only current
        let mut add_current = false;
        for id in &nearest {
            if skip.contains(id.1) {
                continue;
            }
            if let Some(mut n) = self.done.get(id.1) {
                let mut n = n.borrow_mut();
                let (asteps, bsteps) = (p.steps.min(n.steps), p.steps.max(n.steps));
                let psteps = asteps as f32 / bsteps as f32;
                if (psteps < DIVE_DIFF || p.expired || n.expired) && p.adjacent(&n) {
                    if p.expired && n.expired && n.width() < 16.0 {
                        continue;
                    }
                    // println!("{} / {} = {:.3}", asteps, bsteps, psteps);
                    if self.dive.contains(&n.id) {
                        add_current = true;
                        continue;
                    }
                    res.extend(n.split(self.width, MIN_DIVE_PIXEL));
                    skip.insert(n.id);
                    self.dive.insert(n.id);
                    // println!("split {} {}", n.id, self.iter);
                    // println!("split {}", n.id);
                }
            }
        }
        if res.len() > 0 || add_current {
            res.extend(p.split(self.width, MIN_DIVE_PIXEL));
            self.dive.insert(p.id);
        }
        res
    }

    pub fn dive_all(&mut self, ps: &mut Vec<Rc<RefCell<DoublePendulum>>>) -> HashMap<usize, Rc<RefCell<DoublePendulum>>> {
        let mut next = HashMap::new();
        for p in ps {
            let mut p = p.borrow_mut();
            for mut d in self.dive(&mut p) {
                self.counter += 1;
                d.id = self.counter;
                d.parent_id = p.id;
                next.insert(d.id, Rc::new(RefCell::new(d)));
            }
        }
        // println!("[{}] {:?}", self.iter, self.dive);
        next
    }

    fn color_step(&self) -> usize {
        (1.0 / (COLOR_STEP * STEP_DELTA)) as usize
    }

    pub fn update(&mut self, ctx: &mut Context) -> GameResult<()> {
        // let (sender, receiver) = bounded::<DoublePendulum>(self.len());
        // let (res_sender, res_receiver) = bounded::<DoublePendulum>(self.len());
        // let upd_steps = self.update_steps;
        // let max_steps = self.max_steps;
        // let arc_ctx = Arc::new(RefCell::new(ctx));
        // let workers = (1..12).map(|_| {
        //     let receiver = receiver.clone();
        //     let ctx = arc_ctx.clone();
        //     thread::spawn(move || {
        //         while let Ok(p) = receiver.recv() {
        //             p.update(ctx.get_mut(), upd_steps, max_steps);
        //         }
        //     })
        // }).collect::<Vec<_>>();
        let t = Instant::now();
        self.iter += 1;
        let mut next = HashMap::new();
        let mut stopped = Vec::new();
        let color_step = self.color_step();
        for pref in &mut self.ps.values_mut() {
            let mut p = pref.borrow_mut();
            p.update(ctx, self.update_steps, self.max_steps)?;
            if p.stopped {
                stopped.push(pref.clone());

                self.done.insert(p.id, pref.clone());
                self.tree.add(&p.p.to_array(), p.id).unwrap();

                p.update_color(color_step);
                self.to_draw.push_back(pref.clone());

                self.avg.add(p.steps as u32);

                // for mut n in p.split(self.width, MIN_PIXEL) {
                //     self.counter += 1;
                //     n.id = self.counter;
                //     n.parent_id = p.id;
                //     next.insert(n.id, Rc::new(RefCell::new(n)));
                // }
            }
            else {
                next.insert(p.id, pref.clone());
            }
        }
        // println!("[{}] upd1 + {:?}", self.iter, t.elapsed());
        let t = Instant::now();
        if next.len() == 0 && self.done.len() == 1 {
            // special case: only for the first pendulum
            let p = self.done.values().next().unwrap().clone();
            let mut p = p.borrow_mut();
            for mut n in p.split(self.width, MIN_PIXEL) {
                self.counter += 1;
                n.id = self.counter;
                n.parent_id = p.id;
                next.insert(n.id, Rc::new(RefCell::new(n)));
            }
        }
        for p in &stopped {
            let p = p.borrow();
            if self.can_remove(p.parent_id) {
                // println!("remove {}", p.parent_id);
                let parent = self.done[&p.parent_id].clone();
                self.done.remove(&p.parent_id);
                self.tree.remove(&parent.borrow().p.to_array(), &p.parent_id).unwrap();
            }
        }
        // (iter / a).exp() / b: bigger b => longer low values at start
        
        self.update_steps = 10 + ((self.iter as f32 / 550.0).exp() / 20.0).exp() as usize;
        
        //next.extend(self.dive_all(&mut stopped));
        let dive = self.dive_all(&mut stopped);
        next.extend(dive.clone());
        
        if self.ps.len() > 0 {
            println!(
                "[{}] +{}, -{}: active: {}, done: {}, avg: {}, steps: {}",
                self.iter,
                dive.len(),
                stopped.len(),
                next.len(),
                self.done.len(),
                self.avg.get(),
                self.update_steps
            );
        }
        self.ps = next;
        Ok(())
    }

    pub fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
        let t = Instant::now();
        graphics::set_canvas(ctx, Some(&self.canvas));
        while let Some(p) = self.to_draw.pop_front() {
            // println!("draw {}", p.borrow().id);
            p.borrow_mut().draw(ctx)?
        }
        graphics::set_canvas(ctx, None);
        graphics::draw(ctx, &self.canvas, (vec2(0.0, 0.0), (1.0, 1.0, 1.0, 1.0).into()))?;
        // println!("[{}] + {:?}", self.iter, t.elapsed());
        let t = Instant::now();
        for p in self.ps.values() {
            p.borrow_mut().draw(ctx)?
        }
        // println!("[{}] - {:?}", self.iter, t.elapsed());
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
        assert!(p2.adjacent(&p1));
        let mut p3 = p2.clone();
        p3.p.y += 1.0;
        assert!(!p1.adjacent(&p3));
        assert!(!p3.adjacent(&p1));
        let p4 = DoublePendulum::new2(vec2(WIDTH / 4.0 * 3.0, WIDTH / 4.0), WIDTH, 0.5);
        assert!(p1.adjacent(&p4));
        assert!(p4.adjacent(&p1));
    }
}

/*
ffmpeg -r 1/5 -i %06d.png -c:v libx264 -vf fps=25 -pix_fmt yuv420p dpfrac.mp4
*/
