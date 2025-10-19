/**
 * VNC Parser
 * 
 * Parses Van Laarhoven Navigation Calculus expressions and validates syntax
 */

export interface VNCNode {
  type: string;
  value?: any;
  children?: VNCNode[];
  position?: { start: number; end: number };
}

export class VNCParser {
  private source: string;
  private position: number;

  constructor(source: string) {
    this.source = source;
    this.position = 0;
  }

  parse(): VNCNode {
    return this.parseExpression();
  }

  private parseExpression(): VNCNode {
    // Simple parser implementation
    if (this.match('λ')) {
      return this.parseLambda();
    }
    
    if (this.match('⋌')) {
      return this.parseNavigation();
    }
    
    return this.parseAtom();
  }

  private parseLambda(): VNCNode {
    this.consume('λ');
    return {
      type: 'lambda',
      value: this.parseIdentifier()
    };
  }

  private parseNavigation(): VNCNode {
    this.consume('⋌');
    return {
      type: 'navigation',
      value: this.parseIdentifier()
    };
  }

  private parseAtom(): VNCNode {
    const identifier = this.parseIdentifier();
    return {
      type: 'identifier',
      value: identifier
    };
  }

  private parseIdentifier(): string {
    const start = this.position;
    while (this.position < this.source.length && /[a-zA-Z0-9_]/.test(this.source[this.position])) {
      this.position++;
    }
    return this.source.slice(start, this.position);
  }

  private match(char: string): boolean {
    return this.source[this.position] === char;
  }

  private consume(char: string): void {
    if (this.source[this.position] === char) {
      this.position++;
    } else {
      throw new Error(`Expected ${char} at position ${this.position}`);
    }
  }
}

export function parseVNC(source: string): VNCNode {
  const parser = new VNCParser(source);
  return parser.parse();
}