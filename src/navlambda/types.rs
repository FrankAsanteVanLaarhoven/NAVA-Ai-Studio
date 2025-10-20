//! NAVΛ Type System
//! Navigation-specific types and operators

use std::ops::{Add, Sub, Mul, Div};
use std::fmt;

/// NAVΛ primary navigation operator (⋋)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavLambda;

/// Navigation tensor product operator (⊗⋋)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavTensor;

/// Navigation addition operator (⊕⋋)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavAdd;

/// Navigation subtraction operator (⊖⋋)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavSub;

/// Navigation division operator (⊘⋋)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavDiv;

/// Navigation assignment operator (←)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavAssign;

/// 3D Navigation Vector
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Vector3D { x, y, z }
    }

    pub fn magnitude(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn normalize(&self) -> Self {
        let mag = self.magnitude();
        if mag > 0.0 {
            Vector3D {
                x: self.x / mag,
                y: self.y / mag,
                z: self.z / mag,
            }
        } else {
            *self
        }
    }

    pub fn dot(&self, other: &Vector3D) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(&self, other: &Vector3D) -> Vector3D {
        Vector3D {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }
}

impl Add for Vector3D {
    type Output = Vector3D;

    fn add(self, other: Vector3D) -> Vector3D {
        Vector3D {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl Sub for Vector3D {
    type Output = Vector3D;

    fn sub(self, other: Vector3D) -> Vector3D {
        Vector3D {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl Mul<f64> for Vector3D {
    type Output = Vector3D;

    fn mul(self, scalar: f64) -> Vector3D {
        Vector3D {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

/// 7D Spacetime Point (x, y, z, t, G, I, C)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point7D {
    pub x: f64,     // Spatial X
    pub y: f64,     // Spatial Y
    pub z: f64,     // Spatial Z
    pub t: f64,     // Time
    pub goal: f64,  // Goal dimension (G)
    pub intention: f64, // Intention dimension (I)
    pub consciousness: f64, // Consciousness dimension (C)
}

impl Point7D {
    pub fn new(x: f64, y: f64, z: f64, t: f64, goal: f64, intention: f64, consciousness: f64) -> Self {
        Point7D { x, y, z, t, goal, intention, consciousness }
    }

    /// Van Laarhoven 7D spacetime interval
    pub fn spacetime_interval(&self, other: &Point7D) -> f64 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        let dz = other.z - self.z;
        let dt = other.t - self.t;
        let dg = other.goal - self.goal;
        let di = other.intention - self.intention;
        let dc = other.consciousness - self.consciousness;

        // Van Laarhoven metric: ds² = -c²dt² + dx² + dy² + dz² + dG² + dI² + dC²
        let c = 299792458.0; // Speed of light
        -(c * dt).powi(2) + dx.powi(2) + dy.powi(2) + dz.powi(2) +
        dg.powi(2) + di.powi(2) + dc.powi(2)
    }

    /// 7D distance (simplified)
    pub fn distance(&self, other: &Point7D) -> f64 {
        self.spacetime_interval(other).abs().sqrt()
    }
}

/// Navigation Path
#[derive(Debug, Clone)]
pub struct NavigationPath {
    pub waypoints: Vec<Point7D>,
    pub energy_cost: f64,
    pub is_optimal: bool,
}

impl NavigationPath {
    pub fn new() -> Self {
        NavigationPath {
            waypoints: Vec::new(),
            energy_cost: 0.0,
            is_optimal: false,
        }
    }

    pub fn add_waypoint(&mut self, point: Point7D) {
        self.waypoints.push(point);
        self.recalculate_cost();
    }

    pub fn recalculate_cost(&mut self) {
        self.energy_cost = 0.0;
        for i in 0..self.waypoints.len().saturating_sub(1) {
            let interval = self.waypoints[i].spacetime_interval(&self.waypoints[i + 1]);
            self.energy_cost += interval.abs();
        }
    }

    pub fn total_distance(&self) -> f64 {
        if self.waypoints.len() < 2 {
            0.0
        } else {
            self.waypoints[0].distance(&self.waypoints[self.waypoints.len() - 1])
        }
    }
}

/// Navigation Quaternion (for 3D rotations)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavQuaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl NavQuaternion {
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        NavQuaternion { w, x, y, z }
    }

    pub fn identity() -> Self {
        NavQuaternion::new(1.0, 0.0, 0.0, 0.0)
    }

    pub fn normalize(&self) -> Self {
        let norm = (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        if norm > 0.0 {
            NavQuaternion {
                w: self.w / norm,
                x: self.x / norm,
                y: self.y / norm,
                z: self.z / norm,
            }
        } else {
            *self
        }
    }

    pub fn multiply(&self, other: &NavQuaternion) -> NavQuaternion {
        NavQuaternion {
            w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            x: self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y: self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z: self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        }
    }
}

/// Navigation Matrix (for transformations)
#[derive(Debug, Clone)]
pub struct NavMatrix {
    pub rows: usize,
    pub cols: usize,
    pub data: Vec<f64>,
}

impl NavMatrix {
    pub fn new(rows: usize, cols: usize) -> Self {
        NavMatrix {
            rows,
            cols,
            data: vec![0.0; rows * cols],
        }
    }

    pub fn get(&self, row: usize, col: usize) -> f64 {
        self.data[row * self.cols + col]
    }

    pub fn set(&mut self, row: usize, col: usize, value: f64) {
        self.data[row * self.cols + col] = value;
    }

    pub fn multiply(&self, other: &NavMatrix) -> Result<NavMatrix, String> {
        if self.cols != other.rows {
            return Err("Matrix dimension mismatch".to_string());
        }

        let mut result = NavMatrix::new(self.rows, other.cols);

        for i in 0..self.rows {
            for j in 0..other.cols {
                let mut sum = 0.0;
                for k in 0..self.cols {
                    sum += self.get(i, k) * other.get(k, j);
                }
                result.set(i, j, sum);
            }
        }

        Ok(result)
    }
}

/// Navigation Number System (symbolic numbers)
#[derive(Debug, Clone, PartialEq)]
pub enum NavNumber {
    Natural(u64),      // ℕ⋋
    Square(u64),       // □⋋
    Prime(u64),        // Prime⋋
    Perfect(u64),      // Perfect⋋
    Infinity,          // ∞⋋
    Real(f64),
}

impl fmt::Display for NavNumber {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            NavNumber::Natural(n) => write!(f, "{}⋋", n),
            NavNumber::Square(n) => write!(f, "□⋋({})", n),
            NavNumber::Prime(n) => write!(f, "Prime⋋({})", n),
            NavNumber::Perfect(n) => write!(f, "Perfect⋋({})", n),
            NavNumber::Infinity => write!(f, "∞⋋"),
            NavNumber::Real(r) => write!(f, "{}⋋", r),
        }
    }
}

/// Direction operators
pub mod directions {
    use super::Vector3D;

    pub const UP: Vector3D = Vector3D { x: 0.0, y: 0.0, z: 1.0 };
    pub const DOWN: Vector3D = Vector3D { x: 0.0, y: 0.0, z: -1.0 };
    pub const LEFT: Vector3D = Vector3D { x: -1.0, y: 0.0, z: 0.0 };
    pub const RIGHT: Vector3D = Vector3D { x: 1.0, y: 0.0, z: 0.0 };
    pub const FORWARD: Vector3D = Vector3D { x: 0.0, y: 1.0, z: 0.0 };
    pub const BACKWARD: Vector3D = Vector3D { x: 0.0, y: -1.0, z: 0.0 };
}

/// Navigation operators trait
pub trait NavigationOps {
    fn nav_add(&self, other: &Self) -> Self;
    fn nav_sub(&self, other: &Self) -> Self;
    fn nav_mul(&self, scalar: f64) -> Self;
    fn nav_div(&self, scalar: f64) -> Self;
}

impl NavigationOps for Vector3D {
    fn nav_add(&self, other: &Self) -> Self {
        *self + *other
    }

    fn nav_sub(&self, other: &Self) -> Self {
        *self - *other
    }

    fn nav_mul(&self, scalar: f64) -> Self {
        *self * scalar
    }

    fn nav_div(&self, scalar: f64) -> Self {
        Vector3D {
            x: self.x / scalar,
            y: self.y / scalar,
            z: self.z / scalar,
        }
    }
}

impl NavigationOps for Point7D {
    fn nav_add(&self, other: &Self) -> Self {
        Point7D {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
            t: self.t + other.t,
            goal: self.goal + other.goal,
            intention: self.intention + other.intention,
            consciousness: self.consciousness + other.consciousness,
        }
    }

    fn nav_sub(&self, other: &Self) -> Self {
        Point7D {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
            t: self.t - other.t,
            goal: self.goal - other.goal,
            intention: self.intention - other.intention,
            consciousness: self.consciousness - other.consciousness,
        }
    }

    fn nav_mul(&self, scalar: f64) -> Self {
        Point7D {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
            t: self.t * scalar,
            goal: self.goal * scalar,
            intention: self.intention * scalar,
            consciousness: self.consciousness * scalar,
        }
    }

    fn nav_div(&self, scalar: f64) -> Self {
        Point7D {
            x: self.x / scalar,
            y: self.y / scalar,
            z: self.z / scalar,
            t: self.t / scalar,
            goal: self.goal / scalar,
            intention: self.intention / scalar,
            consciousness: self.consciousness / scalar,
        }
    }
}