use std::time::Duration;

use ggez::graphics::{self, Color};
use ggez::*;
use glam::*;

const g: f32 = 9.81;

pub struct DoublePendulum {
    pub p:      Vec2, // fixed point
    pub theta1: f32,  // angle of first arm: 0 .. PHI
    pub theta2: f32,  // angle of snd arm: 0 .. PHI
    pub l1:     f32,  // len of first arm
    pub l2:     f32,  // len of snd arm
    pub dt1:    f32,  //
    pub dt2:    f32,  //
    pub d2t1: f32,
    pub d2t2: f32,
        
}

impl DoublePendulum {
    pub fn update(&mut self, ctx: &mut Context) -> GameResult<()> {
        let a = 2.0 * self.l1 + self.l2 - self.l2 * (2.0 * self.theta1 - 2.0 * self.theta2).cos();
        self.d2t1 = ( 
            -g * (2.0 * self.l1 + self.l2) * self.theta1.sin() 
            - self.l2 * g * (self.theta1 - 2.0 * self.theta2).sin()
            - 2.0*(self.theta1 - self.theta2).sin() * self.l2 
            * (self.dt2 * self.dt2 * self.l2 - self.dt1 * self.dt1 * self.l1 * (self.theta1 - self.theta2).cos())
        ) / (self.l1 * a);
        
        self.d2t2 = ( 2.0 * (self.theta1 - self.theta2).sin()
            * (self.dt1 * self.dt1 * self.l1 *(self.l1 + self.l2) + g*(self.l1 + self.l2))
            * self.theta1.cos() + self.d2t1 * self.l2 * self.l2 * (self.theta1 - self.theta2).cos()
        ) / (self.l2 * a);

        let delta = timer::delta(ctx).as_secs_f32();

        self.dt1 += self.d2t1 * delta;
        self.dt2 += self.d2t2 * delta;
        
        self.theta1 += self.dt1 * delta;
        self.theta2 += self.dt2 * delta;
        Ok(())
    }

    pub fn draw(&self, ctx: &mut Context) -> GameResult<()> {
        let mut p1 = self.p.clone();
        let (ts1, tc1) = self.theta1.sin_cos();
        p1.x += ts1 * self.l1;
        p1.y += tc1 * self.l1;

        let mut p2 = p1.clone();
        let (ts2, tc2) = self.theta2.sin_cos();
        p2.x += ts2 * self.l2;
        p2.y += tc1 * self.l2;

        let mb = &mut graphics::MeshBuilder::new();
        mb.line(&[self.p, p1, p2], 3.0, Color::BLACK)?;
        let mesh = mb.build(ctx)?;
        graphics::draw(ctx, &mesh, (vec2(0.0, 0.0),))?;
        Ok(())
    }
}
/*
    a = (2*m1+m2-m2*Math.cos(2*theta-2*theta2));

    d2theta = (
        -g*(2*m1+m2)*Math.sin(theta)-m2*g*Math.sin(theta-2*theta2)-
        2*Math.sin(theta-theta2)*m2*(dtheta2*dtheta2*l2-dtheta*dtheta*l1*Math.cos(theta-theta2))
    )/(l1*a);

    d2theta2 = (
        2*Math.sin(theta-theta2)*(dtheta*dtheta*l1*(m1+m2)+g*(m1+m2)*
        Math.cos(theta)+dtheta2*dtheta2*l2*m2*Math.cos(theta-theta2))
    )/(l2*a);

    dtheta += d2theta*timeStep;
    dtheta2 += d2theta2*timeStep;
    theta += dtheta*timeStep;
    theta2 += dtheta2*timeStep;
    normalizeAngles();
    $('#theta').val(theta*180/Math.PI);
    $('#theta2').val(theta2*180/Math.PI);
    drawDoublePendulum();
*/
