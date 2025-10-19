//! Physics engine for the NAVΛ simulation platform
//!
//! This module provides a high-performance physics simulation using the Rapier engine,
//! with support for rigid bodies, collisions, joints, and advanced physics features.

use anyhow::Result;
use nalgebra::{Vector3, Isometry3};
use rapier3d::prelude::*;
use crossbeam::channel;

/// Physics simulation engine
pub struct PhysicsEngine {
    /// Rapier physics pipeline
    pipeline: PhysicsPipeline,
    
    /// Physics integration parameters
    integration_parameters: IntegrationParameters,
    
    /// Broad phase for collision detection
    broad_phase: BroadPhase,
    
    /// Narrow phase for collision detection
    narrow_phase: NarrowPhase,
    
    /// Rigid body set
    bodies: RigidBodySet,
    
    /// Collider set
    colliders: ColliderSet,
    
    /// Impulse joint set
    impulse_joints: ImpulseJointSet,
    
    /// Multibody joint set
    multibody_joints: MultibodyJointSet,
    
    /// Simulation island manager
    islands: IslandManager,
    
    /// Collision events channel
    collision_recv: channel::Receiver<CollisionEvent>,
    collision_send: channel::Sender<CollisionEvent>,
    
    /// Contact force events channel
    contact_recv: channel::Receiver<ContactForceEvent>,
    contact_send: channel::Sender<ContactForceEvent>,
}

/// Physical properties of an object
#[derive(Debug, Clone)]
pub struct PhysicalProperties {
    /// Mass of the object (kg)
    pub mass: f32,
    
    /// Friction coefficient
    pub friction: f32,
    
    /// Restitution coefficient
    pub restitution: f32,
    
    /// Linear damping
    pub linear_damping: f32,
    
    /// Angular damping
    pub angular_damping: f32,
}

impl Default for PhysicalProperties {
    fn default() -> Self {
        Self {
            mass: 1.0,
            friction: 0.5,
            restitution: 0.3,
            linear_damping: 0.1,
            angular_damping: 0.1,
        }
    }
}

impl PhysicsEngine {
    /// Create a new physics engine
    pub fn new() -> Self {
        let (collision_send, collision_recv) = channel::unbounded();
        let (contact_send, contact_recv) = channel::unbounded();
        
        Self {
            pipeline: PhysicsPipeline::new(),
            integration_parameters: IntegrationParameters::default(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            islands: IslandManager::new(),
            collision_send,
            collision_recv,
            contact_send,
            contact_recv,
        }
    }
    
    /// Step the physics simulation
    pub fn step(&mut self) {
        let gravity = Vector3::new(0.0, -9.81, 0.0);
        let physics_hooks = ();
        let event_handler = ChannelEventCollector::new(
            self.collision_send.clone(),
            self.contact_send.clone(),
        );
        
        self.pipeline.step(
            &gravity,
            &self.integration_parameters,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut CCDSolver::new(),
            None,
            &physics_hooks,
            &event_handler,
        );
    }
    
    /// Add a rigid body to the simulation
    pub fn add_rigid_body(
        &mut self,
        position: Isometry3<f32>,
        properties: PhysicalProperties,
        shape: SharedShape,
    ) -> RigidBodyHandle {
        let body = RigidBodyBuilder::dynamic()
            .position(position)
            .linear_damping(properties.linear_damping)
            .angular_damping(properties.angular_damping)
            .build();
        
        let handle = self.bodies.insert(body);
        
        let collider = ColliderBuilder::new(shape)
            .mass(properties.mass)
            .friction(properties.friction)
            .restitution(properties.restitution)
            .build();
        
        self.colliders.insert_with_parent(collider, handle, &mut self.bodies);
        
        handle
    }
    
    /// Apply force to a rigid body
    pub fn apply_force(&mut self, handle: RigidBodyHandle, force: Vector3<f32>) {
        if let Some(body) = self.bodies.get_mut(handle) {
            body.add_force(force, true);
        }
    }
    
    /// Get the position of a rigid body
    pub fn get_position(&self, handle: RigidBodyHandle) -> Option<Isometry3<f32>> {
        self.bodies.get(handle).map(|body| *body.position())
    }
}

impl Default for PhysicsEngine {
    fn default() -> Self {
        Self::new()
    }
}

/// Get the gravity vector for the simulation
fn gravity() -> Vector3<f32> {
    Vector3::new(0.0, -9.81, 0.0)
}