//! NAVΛ SDK Rust Example
//!
//! Demonstrates basic usage of the NAVΛ SDK in Rust

use nava_sdk::*;

fn main() -> Result<()> {
    // Initialize SDK
    init();
    println!("NAVΛ SDK v{}", version());

    // Create navigation field
    let mut nav = NavigationField::new();
    
    // Set up 3D Euclidean manifold
    nav.set_manifold(Manifold::euclidean(3));
    
    // Define start and goal
    let start = vec![0.0, 0.0, 0.0];
    let goal = vec![5.0, 5.0, 5.0];
    
    // Set constraints
    let constraints = NavigationConstraints {
        max_velocity: 2.0,
        avoid_obstacles: true,
        custom: None,
    };
    
    // Find optimal path
    println!("Finding optimal path from {:?} to {:?}...", start, goal);
    let path = nav.find_optimal_path(&start, &goal, &constraints)?;
    
    println!("\n✓ Path found!");
    println!("  Waypoints: {}", path.len());
    println!("  Total Energy: {:.2}", path.total_energy);
    println!("  Method: {}", path.optimization_method);
    
    println!("\nPath waypoints:");
    for (i, point) in path.waypoints.iter().enumerate() {
        println!("  {}: {:?} (energy: {:.2})", i, point.coordinates, point.energy);
    }
    
    Ok(())
}

