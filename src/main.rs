use macroquad::prelude::*;

struct Body {
    mass: f32,
    inertia: f32,
    shape: FiniteShape,

    pos: Vec2,
    vel: Vec2,
    force: Vec2,

    angle: f32,
    omega: f32,
    torque: f32,
}

struct Penetration {
    pos: Vec2,
    normal: Vec2,
    depth: f32,
}

enum CollisionResult {
    NoCollision,
    Penetration(Penetration),
    FullOverlap,
}

fn circle_circle_collision(c1: &Circle, c2: &Circle) -> CollisionResult {
    let diff = c1.pos - c2.pos;
    let dist = diff.length();

    if dist > c1.r + c2.r {
        CollisionResult::NoCollision
    } else if dist > c2.r {
        let normal = diff.normalize();
        let pos = c1.pos - c1.r * normal;
        let depth = c1.r + c2.r - dist;
        CollisionResult::Penetration(Penetration { pos, normal, depth })
    } else {
        CollisionResult::FullOverlap
    }
}

fn circle_line_collision(c: &Circle, l: &Line) -> CollisionResult {
    let t = (l.a * c.pos.x + l.b * c.pos.y + l.c) / (l.a * l.a + l.b * l.b);
    let x = l.a * t;
    let y = l.b * t;
    let diff: Vec2 = (x, y).into();
    let dist = diff.length();
    if dist > c.r {
        if t > 0. {
            CollisionResult::NoCollision
        } else {
            CollisionResult::FullOverlap
        }
    } else {
        let normal = diff.normalize();
        let pos = c.pos - c.r * normal;
        let depth = c.r - dist * t.signum();
        CollisionResult::Penetration(Penetration { pos, normal, depth })
    }
}

impl Body {
    fn new(mass: f32, inertia: f32, shape: FiniteShape, pos: Vec2, angle: f32) -> Body {
        Body {
            mass,
            inertia,
            shape,
            pos,
            vel: Vec2::ZERO,
            force: Vec2::ZERO,
            angle,
            omega: 0.,
            torque: 0.,
        }
    }

    fn step(&mut self, dt: f32) {
        let acc = self.force / self.mass;
        self.vel += acc * dt;
        self.pos += self.vel * dt;
        self.force = Vec2::ZERO;

        let alpha = self.torque / self.inertia;
        self.omega += alpha * dt;
        self.angle += self.omega * dt;
        self.torque = 0.;
    }

    fn collide(&mut self, geom: &Geometry) {
        match &self.shape {
            FiniteShape::Circle(c1) => {
                let c = Circle {
                    pos: self.pos,
                    r: c1.r,
                };

                let res = match geom {
                    Geometry::Finite(FiniteShape::Circle(c2)) => circle_circle_collision(&c, c2),
                    Geometry::Infinite(InfiniteShape::Line(l)) => circle_line_collision(&c, l),
                };

                if let CollisionResult::Penetration(Penetration { normal, .. }) = res {
                    // Just reflect the velocity along the penetration normal
                    self.vel -= 2. * self.vel.dot(normal) * normal;
                    //self.pos += depth * n;
                }
            }
        }
    }

    fn draw(&self) {
        match self.shape {
            FiniteShape::Circle(Circle { r, .. }) => {
                draw_circle_lines(self.pos.x, self.pos.y, r, 1.0, WHITE);

                let rot = Mat2::from_angle(self.angle);
                let xdelta = r * rot * Vec2::X;
                let ydelta = r * rot * Vec2::Y;
                draw_line_vec(self.pos + xdelta, self.pos - xdelta);
                draw_line_vec(self.pos + ydelta, self.pos - ydelta);
            }
        }
    }
}

fn draw_line_vec(a: Vec2, b: Vec2) {
    draw_line(a.x, a.y, b.x, b.y, 1.0, WHITE);
}

// ax + by + c == 0
// TODO: disallow constructing invalid lines (a == b == 0)
struct Line {
    a: f32,
    b: f32,
    c: f32,
}

impl Line {
    // TODO: line limits should depend on window size
    fn draw(&self) {
        if self.b == 0. {
            assert!(self.a != 0.);
            let x = -self.c / self.a;
            let y1 = -10.;
            let y2 = 2000.;
            draw_line(x, y1, x, y2, 1., WHITE);
        } else {
            let x1 = -10.;
            let y1 = -(self.c + self.a * x1) / self.b;
            let x2 = 2000.;
            let y2 = -(self.c + self.a * x2) / self.b;
            draw_line(x1, y1, x2, y2, 1., WHITE);
        }
    }
}

struct Circle {
    pos: Vec2,
    r: f32,
}

impl Circle {
    fn draw(&self) {
        draw_circle_lines(self.pos.x, self.pos.y, self.r, 1., WHITE)
    }
}

enum InfiniteShape {
    Line(Line),
}

impl InfiniteShape {
    fn draw(&self) {
        match self {
            InfiniteShape::Line(line) => line.draw(),
        }
    }
}

enum FiniteShape {
    Circle(Circle),
}

impl FiniteShape {
    fn draw(&self) {
        match self {
            FiniteShape::Circle(circle) => circle.draw(),
        }
    }
}

enum Geometry {
    Infinite(InfiniteShape),
    Finite(FiniteShape),
}

impl Geometry {
    fn draw(&self) {
        match self {
            Geometry::Infinite(shape) => shape.draw(),
            Geometry::Finite(shape) => shape.draw(),
        }
    }
}

// TODO: use Geometry
struct State {
    geometry: Vec<Geometry>,
    bodies: Vec<Body>,
}

impl State {
    fn set_gravity(&mut self, f: Vec2) {
        for b in &mut self.bodies {
            b.force += f;
        }
    }

    fn collide(&mut self) {
        for b in &mut self.bodies {
            for g in &self.geometry {
                b.collide(g);
            }
        }
    }

    fn step(&mut self, dt: f32) {
        for b in &mut self.bodies {
            b.step(dt);
        }

        self.collide();
    }

    fn draw(&self) {
        for shape in &self.geometry {
            shape.draw();
        }

        for b in &self.bodies {
            b.draw();
        }
    }
}

#[macroquad::main("Pheng")]
async fn main() {
    let pos: Vec2 = (500., 200.).into();
    let circ = Circle {
        pos: Vec2::ZERO,
        r: 20.,
    };
    let body = Body::new(1., 1000., FiniteShape::Circle(circ), pos, 1.);
    let line = Line {
        a: 0.,
        b: -1.,
        c: 500.,
    };
    let line2 = Line {
        a: 1.,
        b: 0.,
        c: 0.,
    };
    let mut state = State {
        geometry: vec![
            Geometry::Infinite(InfiniteShape::Line(line)),
            Geometry::Infinite(InfiniteShape::Line(line2)),
            Geometry::Finite(FiniteShape::Circle(Circle {
                pos: (600., 400.).into(),
                r: 150.,
            })),
        ],
        bodies: vec![body],
    };

    loop {
        state.draw();

        let dt = get_frame_time();

        state.set_gravity((0., 200.).into());
        state.step(dt);

        next_frame().await;
    }
}
