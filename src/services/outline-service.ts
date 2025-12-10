/**
 * Outline Service
 * Parses code to extract structure (functions, variables, classes, etc.)
 */

export interface OutlineItem {
  name: string;
  type: 'function' | 'variable' | 'class' | 'interface' | 'navigation' | 'operator' | 'constant' | 'import';
  line: number;
  children?: OutlineItem[];
  icon?: string;
}

class OutlineService {
  /**
   * Parse code and extract outline structure
   */
  parseCode(code: string, language?: string): OutlineItem[] {
    if (!code || !code.trim()) {
      return [];
    }

    const items: OutlineItem[] = [];
    const lines = code.split('\n');

    // Detect language if not provided
    const detectedLang = language || this.detectLanguage(code);

    switch (detectedLang) {
      case 'navlambda':
      case 'vnc':
        return this.parseNavLambda(code, lines);
      case 'typescript':
      case 'javascript':
        return this.parseJavaScript(code, lines);
      case 'python':
        return this.parsePython(code, lines);
      case 'rust':
        return this.parseRust(code, lines);
      default:
        return this.parseGeneric(code, lines);
    }
  }

  /**
   * Detect programming language from code
   */
  private detectLanguage(code: string): string {
    if (code.includes('⋋') || code.includes('program') || code.includes('let⋋')) {
      return 'navlambda';
    }
    if (code.includes('function') || code.includes('const ') || code.includes('export')) {
      return 'typescript';
    }
    if (code.includes('def ') || code.includes('import ') || code.includes('class ')) {
      return 'python';
    }
    if (code.includes('fn ') || code.includes('pub ') || code.includes('struct ')) {
      return 'rust';
    }
    return 'typescript';
  }

  /**
   * Parse NAVΛ/VNC code
   */
  private parseNavLambda(code: string, lines: string[]): OutlineItem[] {
    const items: OutlineItem[] = [];

    lines.forEach((line, index) => {
      const lineNum = index + 1;
      const trimmed = line.trim();

      // Variables: let⋋ name = ...
      const varMatch = trimmed.match(/let⋋\s+(\w+)/);
      if (varMatch) {
        items.push({
          name: varMatch[1],
          type: 'variable',
          line: lineNum,
          icon: 'v',
        });
      }

      // Functions: function⋋ name(...) or name⋋ = ...
      const funcMatch = trimmed.match(/(?:function⋋|fn⋋)\s+(\w+)/) || trimmed.match(/(\w+)⋋\s*=/);
      if (funcMatch && !varMatch) {
        items.push({
          name: funcMatch[1],
          type: 'function',
          line: lineNum,
          icon: 'f',
        });
      }

      // Navigation: ⋋ name or name⋋
      const navMatch = trimmed.match(/⋋\s*(\w+)/) || trimmed.match(/(\w+)⋋/);
      if (navMatch && !varMatch && !funcMatch) {
        items.push({
          name: navMatch[1],
          type: 'navigation',
          line: lineNum,
          icon: '⋋',
        });
      }

      // Operators: 𝒩ℐ, ⊕⋋, ⊗⋋
      if (trimmed.includes('𝒩ℐ') || trimmed.includes('⊕⋋') || trimmed.includes('⊗⋋')) {
        items.push({
          name: trimmed.match(/[𝒩ℐ⊕⊗⋋]+/)?.[0] || 'operator',
          type: 'operator',
          line: lineNum,
          icon: '𝒩',
        });
      }
    });

    return items;
  }

  /**
   * Parse JavaScript/TypeScript code
   */
  private parseJavaScript(code: string, lines: string[]): OutlineItem[] {
    const items: OutlineItem[] = [];

    lines.forEach((line, index) => {
      const lineNum = index + 1;
      const trimmed = line.trim();

      // Functions: function name, const name = () =>, export function
      const funcMatch = trimmed.match(/(?:export\s+)?(?:async\s+)?function\s+(\w+)/) ||
                       trimmed.match(/(?:export\s+)?const\s+(\w+)\s*=\s*(?:\(|async\s*\()/) ||
                       trimmed.match(/(?:export\s+)?(\w+)\s*:\s*(?:\(|\([^)]*\)\s*=>)/);
      if (funcMatch) {
        items.push({
          name: funcMatch[1],
          type: 'function',
          line: lineNum,
          icon: 'f',
        });
      }

      // Classes: class Name
      const classMatch = trimmed.match(/(?:export\s+)?class\s+(\w+)/);
      if (classMatch) {
        items.push({
          name: classMatch[1],
          type: 'class',
          line: lineNum,
          icon: 'C',
        });
      }

      // Interfaces: interface Name
      const interfaceMatch = trimmed.match(/(?:export\s+)?interface\s+(\w+)/);
      if (interfaceMatch) {
        items.push({
          name: interfaceMatch[1],
          type: 'interface',
          line: lineNum,
          icon: 'I',
        });
      }

      // Variables: const name =, let name =, var name =
      if (!funcMatch && !classMatch && !interfaceMatch) {
        const varMatch = trimmed.match(/(?:export\s+)?(?:const|let|var)\s+(\w+)/);
        if (varMatch) {
          items.push({
            name: varMatch[1],
            type: 'variable',
            line: lineNum,
            icon: 'v',
          });
        }
      }

      // Imports: import ... from
      const importMatch = trimmed.match(/import\s+(?:\{([^}]+)\}|(\w+)|.*?)\s+from/);
      if (importMatch) {
        const imports = importMatch[1]?.split(',').map(i => i.trim()) || [importMatch[2]];
        imports.forEach(imp => {
          if (imp) {
            items.push({
              name: imp,
              type: 'import',
              line: lineNum,
              icon: '>',
            });
          }
        });
      }
    });

    return items;
  }

  /**
   * Parse Python code
   */
  private parsePython(code: string, lines: string[]): OutlineItem[] {
    const items: OutlineItem[] = [];

    lines.forEach((line, index) => {
      const lineNum = index + 1;
      const trimmed = line.trim();

      // Functions: def name
      const funcMatch = trimmed.match(/def\s+(\w+)/);
      if (funcMatch) {
        items.push({
          name: funcMatch[1],
          type: 'function',
          line: lineNum,
          icon: 'f',
        });
      }

      // Classes: class Name
      const classMatch = trimmed.match(/class\s+(\w+)/);
      if (classMatch) {
        items.push({
          name: classMatch[1],
          type: 'class',
          line: lineNum,
          icon: 'C',
        });
      }

      // Variables: name = (not in function/class)
      if (!funcMatch && !classMatch) {
        const varMatch = trimmed.match(/^(\w+)\s*=/);
        if (varMatch) {
          items.push({
            name: varMatch[1],
            type: 'variable',
            line: lineNum,
            icon: 'v',
          });
        }
      }
    });

    return items;
  }

  /**
   * Parse Rust code
   */
  private parseRust(code: string, lines: string[]): OutlineItem[] {
    const items: OutlineItem[] = [];

    lines.forEach((line, index) => {
      const lineNum = index + 1;
      const trimmed = line.trim();

      // Functions: fn name or pub fn name
      const funcMatch = trimmed.match(/(?:pub\s+)?fn\s+(\w+)/);
      if (funcMatch) {
        items.push({
          name: funcMatch[1],
          type: 'function',
          line: lineNum,
          icon: 'f',
        });
      }

      // Structs: struct Name or pub struct Name
      const structMatch = trimmed.match(/(?:pub\s+)?struct\s+(\w+)/);
      if (structMatch) {
        items.push({
          name: structMatch[1],
          type: 'class',
          line: lineNum,
          icon: 'S',
        });
      }

      // Constants: const NAME or pub const NAME
      const constMatch = trimmed.match(/(?:pub\s+)?const\s+(\w+)/);
      if (constMatch) {
        items.push({
          name: constMatch[1],
          type: 'constant',
          line: lineNum,
          icon: 'c',
        });
      }
    });

    return items;
  }

  /**
   * Generic parser for unknown languages
   */
  private parseGeneric(code: string, lines: string[]): OutlineItem[] {
    const items: OutlineItem[] = [];

    lines.forEach((line, index) => {
      const lineNum = index + 1;
      const trimmed = line.trim();

      // Look for common patterns
      if (trimmed.match(/^(function|def|fn)\s+\w+/)) {
        const match = trimmed.match(/(?:function|def|fn)\s+(\w+)/);
        if (match) {
          items.push({
            name: match[1],
            type: 'function',
            line: lineNum,
            icon: 'f',
          });
        }
      }
    });

    return items;
  }
}

export const outlineService = new OutlineService();

