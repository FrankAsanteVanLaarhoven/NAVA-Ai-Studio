/**
 * Parser Types
 * 
 * Type definitions for the VNC parser
 */

export interface VNCNode {
  type: string;
  value?: any;
  children?: VNCNode[];
  position?: { start: number; end: number };
}

export interface ParserError {
  message: string;
  position: number;
  code: string;
}

export interface ParseResult {
  ast: VNCNode;
  errors: ParserError[];
  warnings: ParserError[];
}