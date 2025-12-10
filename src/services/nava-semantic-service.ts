/**
 * NAVA Semantic Intelligence Service
 * 
 * Provides AST parsing, symbol resolution, go-to-definition, and semantic analysis
 * for NAVA code in Monaco Editor
 */

export interface ASTNode {
  type: 'program' | 'declaration' | 'expression' | 'function_call' | 'variable' | 'manifold' | 'field' | 'obstacle' | 'path';
  name?: string;
  value?: any;
  children?: ASTNode[];
  range?: { start: number; end: number; line: number; column: number };
  documentation?: string;
}

export interface Symbol {
  name: string;
  type: 'variable' | 'function' | 'manifold' | 'field' | 'obstacle' | 'path';
  definition: ASTNode;
  references: Array<{ line: number; column: number }>;
  documentation?: string;
}

export interface SemanticInfo {
  ast: ASTNode;
  symbols: Map<string, Symbol>;
  errors: Array<{ message: string; line: number; column: number }>;
  warnings: Array<{ message: string; line: number; column: number }>;
}

class NAVASemanticService {
  /**
   * Parse NAVA code into AST
   */
  parse(code: string): ASTNode {
    const lines = code.split('\n');
    const root: ASTNode = {
      type: 'program',
      children: [],
      range: { start: 0, end: code.length, line: 0, column: 0 },
    };

    let offset = 0;
    for (let lineNum = 0; lineNum < lines.length; lineNum++) {
      const line = lines[lineNum];
      const trimmed = line.trim();

      // Skip empty lines and comments
      if (!trimmed || trimmed.startsWith('//')) {
        offset += line.length + 1;
        continue;
      }

      // Parse variable declarations: let name = ...
      const letMatch = trimmed.match(/^let\s+(\w+)\s*=\s*(.+)$/);
      if (letMatch) {
        const [, name, value] = letMatch;
        const node: ASTNode = {
          type: 'declaration',
          name,
          value: this.parseValue(value),
          range: {
            start: offset,
            end: offset + line.length,
            line: lineNum,
            column: line.indexOf('let'),
          },
        };

        // Determine type from value
        if (value.includes('euclidean_plane') || value.includes('euclidean_space') || value.includes('circle') || value.includes('sphere') || value.includes('se2') || value.includes('se3')) {
          node.type = 'manifold';
        } else if (value.includes('navigation_field')) {
          node.type = 'field';
        } else if (value.includes('disc') || value.includes('rectangle') || value.includes('polygon')) {
          node.type = 'obstacle';
        } else if (value.includes('compute_optimal_path')) {
          node.type = 'path';
        } else if (value.includes('(') && value.includes(')')) {
          node.type = 'function_call';
        } else {
          node.type = 'variable';
        }

        root.children?.push(node);
      }

      offset += line.length + 1;
    }

    return root;
  }

  /**
   * Parse a value expression
   */
  private parseValue(value: string): any {
    value = value.trim();

    // Array literal
    if (value.startsWith('[') && value.endsWith(']')) {
      return JSON.parse(value);
    }

    // Function call
    if (value.includes('(') && value.includes(')')) {
      const match = value.match(/(\w+)\s*\(/);
      return {
        type: 'function_call',
        name: match?.[1] || 'unknown',
        args: value,
      };
    }

    // String or number
    if (value.match(/^-?\d+\.?\d*$/)) {
      return parseFloat(value);
    }

    if (value.startsWith('"') || value.startsWith("'")) {
      return value.slice(1, -1);
    }

    // Variable reference
    return { type: 'reference', name: value };
  }

  /**
   * Build symbol table from AST
   */
  buildSymbolTable(ast: ASTNode): Map<string, Symbol> {
    const symbols = new Map<string, Symbol>();

    const traverse = (node: ASTNode) => {
      if (node.type === 'declaration' && node.name) {
        const symbol: Symbol = {
          name: node.name,
          type: node.type === 'manifold' ? 'manifold' :
                node.type === 'field' ? 'field' :
                node.type === 'obstacle' ? 'obstacle' :
                node.type === 'path' ? 'path' :
                node.type === 'function_call' ? 'function' : 'variable',
          definition: node,
          references: [],
          documentation: node.documentation,
        };
        symbols.set(node.name, symbol);
      }

      if (node.children) {
        node.children.forEach(traverse);
      }
    };

    traverse(ast);

    // Find references
    const code = this.astToCode(ast);
    symbols.forEach((symbol, name) => {
      const regex = new RegExp(`\\b${name}\\b`, 'g');
      let match;
      let lineNum = 0;
      let column = 0;
      const lines = code.split('\n');

      while ((match = regex.exec(code)) !== null) {
        // Find line and column
        let currentOffset = 0;
        for (let i = 0; i < lines.length; i++) {
          if (currentOffset + lines[i].length >= match.index!) {
            lineNum = i;
            column = match.index! - currentOffset;
            break;
          }
          currentOffset += lines[i].length + 1;
        }

        // Skip if this is the definition
        if (symbol.definition.range &&
            lineNum === symbol.definition.range.line &&
            column === symbol.definition.range.column) {
          continue;
        }

        symbol.references.push({ line: lineNum, column });
      }
    });

    return symbols;
  }

  /**
   * Get semantic information for code
   */
  getSemanticInfo(code: string): SemanticInfo {
    const ast = this.parse(code);
    const symbols = this.buildSymbolTable(ast);
    const errors: Array<{ message: string; line: number; column: number }> = [];
    const warnings: Array<{ message: string; line: number; column: number }> = [];

    // Validate: check for undefined references
    const codeLines = code.split('\n');
    symbols.forEach((symbol, name) => {
      // Check if variable is used before definition
      symbol.references.forEach(ref => {
        if (ref.line < symbol.definition.range!.line) {
          warnings.push({
            message: `Variable '${name}' used before definition`,
            line: ref.line,
            column: ref.column,
          });
        }
      });
    });

    // Validate: check for required components
    const hasManifold = Array.from(symbols.values()).some(s => s.type === 'manifold');
    const hasField = Array.from(symbols.values()).some(s => s.type === 'field');
    const hasPath = Array.from(symbols.values()).some(s => s.type === 'path');

    if (!hasManifold) {
      warnings.push({
        message: 'No manifold defined. Consider adding: let M = euclidean_plane()',
        line: 0,
        column: 0,
      });
    }

    if (!hasField) {
      warnings.push({
        message: 'No navigation field defined. Consider adding: let field = navigation_field(...)',
        line: 0,
        column: 0,
      });
    }

    if (!hasPath) {
      warnings.push({
        message: 'No path computation. Consider adding: let path = compute_optimal_path(field)',
        line: 0,
        column: 0,
      });
    }

    return {
      ast,
      symbols,
      errors,
      warnings,
    };
  }

  /**
   * Find definition of symbol at position
   */
  findDefinition(code: string, line: number, column: number): Symbol | null {
    const semantic = this.getSemanticInfo(code);
    const lines = code.split('\n');
    const lineText = lines[line] || '';
    
    // Find word at position
    const before = lineText.substring(0, column);
    const after = lineText.substring(column);
    const beforeMatch = before.match(/(\w+)$/);
    const afterMatch = after.match(/^(\w+)/);
    
    const word = (beforeMatch?.[1] || '') + (afterMatch?.[1] || '');
    
    if (word && semantic.symbols.has(word)) {
      return semantic.symbols.get(word)!;
    }

    return null;
  }

  /**
   * Find all references to a symbol
   */
  findReferences(code: string, symbolName: string): Array<{ line: number; column: number }> {
    const semantic = this.getSemanticInfo(code);
    const symbol = semantic.symbols.get(symbolName);
    return symbol?.references || [];
  }

  /**
   * Get hover information at position
   */
  getHoverInfo(code: string, line: number, column: number): string | null {
    const symbol = this.findDefinition(code, line, column);
    if (!symbol) return null;

    let info = `**${symbol.name}** (${symbol.type})\n\n`;
    
    if (symbol.documentation) {
      info += symbol.documentation + '\n\n';
    }

    info += `Defined at line ${symbol.definition.range!.line + 1}\n`;
    info += `Referenced ${symbol.references.length} time(s)`;

    return info;
  }

  /**
   * Get completion suggestions at position
   */
  getCompletions(code: string, line: number, column: number): Array<{ label: string; kind: string; detail: string; insertText: string }> {
    const semantic = this.getSemanticInfo(code);
    const completions: Array<{ label: string; kind: string; detail: string; insertText: string }> = [];

    // Add symbols
    semantic.symbols.forEach((symbol, name) => {
      completions.push({
        label: name,
        kind: symbol.type,
        detail: `${symbol.type}: ${name}`,
        insertText: name,
      });
    });

    // Add NAVA built-ins
    const builtins = [
      { label: 'euclidean_plane', kind: 'function', detail: 'Creates a 2D Euclidean plane manifold', insertText: 'euclidean_plane()' },
      { label: 'euclidean_space', kind: 'function', detail: 'Creates a 3D Euclidean space manifold', insertText: 'euclidean_space(dim: 3)' },
      { label: 'navigation_field', kind: 'function', detail: 'Creates a navigation field', insertText: 'navigation_field(manifold: M, start: start, goal: goal, obstacles: [], cost: geodesic_cost())' },
      { label: 'compute_optimal_path', kind: 'function', detail: 'Computes optimal path from navigation field', insertText: 'compute_optimal_path(field)' },
      { label: 'disc', kind: 'function', detail: 'Creates a circular obstacle', insertText: 'disc(center: [0.0, 0.0], radius: 1.0)' },
      { label: 'geodesic_cost', kind: 'function', detail: 'Cost function for geodesic (shortest) path', insertText: 'geodesic_cost()' },
    ];

    completions.push(...builtins);

    return completions;
  }

  /**
   * Convert AST back to code (for refactoring)
   */
  private astToCode(ast: ASTNode): string {
    // Simplified - in production would reconstruct full code
    return '';
  }
}

export const navaSemanticService = new NAVASemanticService();

