/**
 * Terminal Types
 * 
 * Type definitions for the VNC terminal
 */

export interface TerminalSession {
  id: string;
  executeROSCommand(command: string): Promise<CommandResult>;
  executeVNCScript(script: string): Promise<CommandResult>;
  close(): void;
  isConnected(): boolean;
}

export interface CommandResult {
  success: boolean;
  output: string;
  errors: string[];
}

export interface ROSCommand {
  command: string;
  args: string[];
  description: string;
  category: string;
}