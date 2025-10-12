/**
 * WebRTC Collaboration Service
 * 
 * Real-time collaborative editing using WebRTC
 */

interface CollaborationPeer {
  id: string;
  name: string;
  color: string;
  cursor?: { line: number; column: number };
}

interface CollaborationMessage {
  type: 'cursor' | 'edit' | 'selection' | 'join' | 'leave';
  peer: string;
  data: any;
  timestamp: number;
}

class CollaborationService {
  private peers: Map<string, RTCPeerConnection> = new Map();
  private dataChannels: Map<string, RTCDataChannel> = new Map();
  private localPeer: CollaborationPeer | null = null;
  private activePeers: Map<string, CollaborationPeer> = new Map();
  private onMessageCallbacks: ((message: CollaborationMessage) => void)[] = [];

  async initialize(userName: string): Promise<void> {
    this.localPeer = {
      id: this.generatePeerId(),
      name: userName,
      color: this.generateRandomColor(),
    };

    console.log('Collaboration initialized:', this.localPeer);
  }

  async createRoom(roomId: string): Promise<string> {
    // In production, this would connect to a signaling server
    console.log('Creating collaboration room:', roomId);
    return roomId;
  }

  async joinRoom(roomId: string): Promise<void> {
    // In production, this would establish WebRTC connections
    console.log('Joining collaboration room:', roomId);
  }

  async sendCursorPosition(line: number, column: number): Promise<void> {
    const message: CollaborationMessage = {
      type: 'cursor',
      peer: this.localPeer?.id || '',
      data: { line, column },
      timestamp: Date.now(),
    };

    this.broadcast(message);
  }

  async sendEdit(edit: any): Promise<void> {
    const message: CollaborationMessage = {
      type: 'edit',
      peer: this.localPeer?.id || '',
      data: edit,
      timestamp: Date.now(),
    };

    this.broadcast(message);
  }

  onMessage(callback: (message: CollaborationMessage) => void): void {
    this.onMessageCallbacks.push(callback);
  }

  getActivePeers(): CollaborationPeer[] {
    return Array.from(this.activePeers.values());
  }

  private broadcast(message: CollaborationMessage): void {
    this.dataChannels.forEach((channel) => {
      if (channel.readyState === 'open') {
        channel.send(JSON.stringify(message));
      }
    });

    // Also trigger local callbacks for testing
    this.onMessageCallbacks.forEach((callback) => callback(message));
  }

  private generatePeerId(): string {
    return `peer-${Math.random().toString(36).substr(2, 9)}`;
  }

  private generateRandomColor(): string {
    const colors = ['#00ff00', '#00ccff', '#ff0099', '#ffff00', '#ff6600'];
    return colors[Math.floor(Math.random() * colors.length)];
  }

  leaveRoom(): void {
    this.peers.forEach((peer) => peer.close());
    this.peers.clear();
    this.dataChannels.clear();
    this.activePeers.clear();
  }
}

export const collaborationService = new CollaborationService();

