/**
 * NAVA Explanation Service
 * 
 * Generates educational explanations at multiple levels
 */

export class NAVAExplanationService {
  async generateExplanation(
    code: string,
    level: 'gcse' | 'alevel' | 'undergrad' | 'phd' = 'alevel',
    focus?: string
  ): Promise<string> {
    // In production, this would call the Python service or use a TypeScript implementation
    // For now, use a simplified version
    
    const conceptTemplates: Record<string, Record<string, string>> = {
      manifold: {
        gcse: 'the space or surface we\'re navigating on (like a flat map or a curved surface)',
        alevel: 'the configuration space or manifold where navigation occurs (e.g., R^2 for the plane, S^1 for a circle)',
        undergrad: 'the Riemannian manifold that defines the state space, with its metric determining distances',
        phd: 'the differentiable manifold M with metric tensor g, defining the configuration space and geodesic structure',
      },
      navigation_field: {
        gcse: 'an invisible force field that points towards the goal and pushes away from obstacles',
        alevel: 'a vector field on the manifold that encodes the navigation policy, combining attractive forces to the goal and repulsive forces from obstacles',
        undergrad: 'a potential field or vector field on the manifold whose gradient guides optimal trajectories',
        phd: 'a smooth vector field X: M → TM that encodes the navigation policy, typically derived from a potential function V: M → R with X = -∇V',
      },
    };

    const lines = code.split('\n').filter(l => l.trim() && !l.trim().startsWith('//'));
    
    if (level === 'gcse') {
      return `This code finds a path from one point to another on a flat surface. ${lines.some(l => l.includes('obstacle')) ? 'There are obstacles in the way that the path must avoid. ' : ''}The navigation field is like an invisible guide that points towards the goal. The code then finds the best path that follows this guide.`;
    } else if (level === 'alevel') {
      return `This NAVA code sets up a navigation problem on a manifold. ${conceptTemplates.manifold[level]}. ${conceptTemplates.navigation_field[level]}. The code computes an optimal path that minimizes the cost while following the navigation field.`;
    } else if (level === 'undergrad') {
      return `This implements a path planning problem on a Riemannian manifold. ${conceptTemplates.navigation_field[level]}. The optimal path is found by solving the variational problem that minimizes the action functional.`;
    } else {
      return `This NAVA program formulates an optimal control problem on a differentiable manifold M. ${conceptTemplates.navigation_field[level]}. The optimal trajectory γ*: [0,T] → M satisfies the Euler-Lagrange equations for the Lagrangian L.`;
    }
  }

  async explainLine(
    line: string,
    level: 'gcse' | 'alevel' | 'undergrad' | 'phd' = 'alevel'
  ): Promise<string> {
    line = line.trim();

    if (line.includes('euclidean_plane')) {
      return level === 'gcse' ? 'Creates a flat 2D surface for navigation' :
             level === 'alevel' ? 'Defines the Euclidean plane R^2 as the navigation manifold' :
             level === 'undergrad' ? 'Instantiates the 2D Euclidean manifold with standard metric' :
             'Defines the Riemannian manifold (R^2, g_E) where g_E is the Euclidean metric';
    } else if (line.includes('navigation_field')) {
      return level === 'gcse' ? 'Creates a guide that points to the goal' :
             level === 'alevel' ? 'Builds a vector field that guides navigation' :
             level === 'undergrad' ? 'Constructs a potential field whose gradient defines the navigation policy' :
             'Defines the navigation field X: M → TM from potential V';
    } else if (line.includes('compute_optimal_path')) {
      return level === 'gcse' ? 'Finds the best path following the guide' :
             level === 'alevel' ? 'Computes the optimal trajectory' :
             level === 'undergrad' ? 'Solves the optimal control problem' :
             'Finds γ* that minimizes J[γ] subject to dynamics';
    }

    return '';
  }
}

export const navaExplanationGenerator = new NAVAExplanationService();

