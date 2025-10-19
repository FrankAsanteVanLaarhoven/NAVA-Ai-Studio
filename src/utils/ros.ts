/**
 * ROS Utilities
 * 
 * Utility functions for ROS integration
 */

export interface ROSNode {
  name: string;
  type: string;
  status: 'active' | 'inactive' | 'error';
}

export interface ROSTopic {
  name: string;
  type: string;
  publishers: number;
  subscribers: number;
}

export interface ROSService {
  name: string;
  type: string;
  providers: number;
}

export function parseROSCommand(command: string): ROSCommand {
  const parts = command.trim().split(/\s+/);
  const cmd = parts[0];
  const args = parts.slice(1);
  
  return {
    command: cmd,
    args,
    description: '',
    category: 'user'
  };
}

export interface ROSCommand {
  command: string;
  args: string[];
  description: string;
  category: string;
}