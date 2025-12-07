/**
 * NAVΛ Studio Event Bus
 * 
 * Central event bus for inter-app communication
 */

export type Message = {
  type: string;
  payload?: any;
  sender?: string;
  target?: string;
  timestamp?: number;
};

type MessageHandler = (message: Message) => void;

export class Bus {
  private handlers: Map<string, Set<MessageHandler>> = new Map();
  private globalHandlers: Set<MessageHandler> = new Set();

  /**
   * Subscribe to a specific message type
   */
  on(type: string, handler: MessageHandler): () => void {
    if (!this.handlers.has(type)) {
      this.handlers.set(type, new Set());
    }
    this.handlers.get(type)!.add(handler);

    // Return unsubscribe function
    return () => {
      this.handlers.get(type)?.delete(handler);
    };
  }

  /**
   * Subscribe to all messages
   */
  onAll(handler: MessageHandler): () => void {
    this.globalHandlers.add(handler);
    return () => {
      this.globalHandlers.delete(handler);
    };
  }

  /**
   * Send a message
   */
  send(message: Message): void {
    const fullMessage: Message = {
      ...message,
      timestamp: message.timestamp || Date.now(),
    };

    // Call type-specific handlers
    const typeHandlers = this.handlers.get(message.type);
    if (typeHandlers) {
      typeHandlers.forEach(handler => {
        try {
          handler(fullMessage);
        } catch (error) {
          console.error(`[Bus] Error in handler for ${message.type}:`, error);
        }
      });
    }

    // Call global handlers
    this.globalHandlers.forEach(handler => {
      try {
        handler(fullMessage);
      } catch (error) {
        console.error('[Bus] Error in global handler:', error);
      }
    });
  }

  /**
   * Clear all handlers
   */
  clear(): void {
    this.handlers.clear();
    this.globalHandlers.clear();
  }

  /**
   * Get statistics
   */
  getStats() {
    return {
      typeHandlers: this.handlers.size,
      globalHandlers: this.globalHandlers.size,
      totalHandlers: Array.from(this.handlers.values()).reduce((sum, set) => sum + set.size, 0) + this.globalHandlers.size,
    };
  }
}

export default Bus;

