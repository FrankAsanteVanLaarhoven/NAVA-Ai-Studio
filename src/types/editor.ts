/**
 * Editor Types
 * 
 * Type definitions for the VNC editor
 */

export interface EditorOptions {
  language: string;
  theme: string;
  fontSize: number;
  vncMode: boolean;
}

export interface EditorAPI {
  getValue(): string;
  setValue(value: string): void;
  getOption(option: string): any;
  setOption(option: string, value: any): void;
  dispose(): void;
  onDidChangeModelContent(listener: () => void): { dispose: () => void };
  onDidChangeCursorPosition(listener: () => void): { dispose: () => void };
  updateOptions(newOptions: any): void;
}

export interface VNCCompletionProvider {
  provideCompletionItems(model: any, position: any, context: any, token: any): Promise<any>;
}

export interface TokenDefinition {
  token: string;
  definition: string;
  documentation: string;
}