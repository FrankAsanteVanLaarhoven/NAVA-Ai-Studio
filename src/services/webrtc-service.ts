/**
 * WebRTC Service for NAVΛ Studio IDE
 * 
 * Provides real-time communication capabilities for:
 * - Collaborative coding sessions
 * - Screen sharing for navigation visualization
 * - Voice/video chat for team collaboration
 * - Real-time data synchronization
 */

export interface WebRTCConfig {
  iceServers: RTCIceServer[];
  enableAudio: boolean;
  enableVideo: boolean;
  enableDataChannel: boolean;
}

export interface NavigationDataChannel {
  send(data: any): void;
  onMessage(callback: (data: any) => void): void;
  close(): void;
}

class WebRTCService {
  private peerConnection: RTCPeerConnection | null = null;
  private dataChannel: RTCDataChannel | null = null;
  private localStream: MediaStream | null = null;
  private remoteStream: MediaStream | null = null;
  private config: WebRTCConfig;

  constructor() {
    this.config = {
      iceServers: [
        { urls: 'stun:stun.l.google.com:19302' },
        { urls: 'stun:stun1.l.google.com:19302' },
        { urls: 'stun:stun2.l.google.com:19302' },
      ],
      enableAudio: true,
      enableVideo: true,
      enableDataChannel: true,
    };
  }

  /**
   * Initialize WebRTC connection
   */
  async initialize(): Promise<void> {
    try {
      this.peerConnection = new RTCPeerConnection({
        iceServers: this.config.iceServers,
      });

      this.setupPeerConnectionHandlers();
      
      if (this.config.enableDataChannel) {
        this.createDataChannel();
      }

      console.log('⋋ WebRTC Service: Initialized');
    } catch (error) {
      console.error('⋋ WebRTC Service: Initialization error', error);
      throw error;
    }
  }

  /**
   * Setup peer connection event handlers
   */
  private setupPeerConnectionHandlers(): void {
    if (!this.peerConnection) return;

    this.peerConnection.onicecandidate = (event) => {
      if (event.candidate) {
        console.log('⋋ WebRTC: ICE candidate', event.candidate);
        // Send ICE candidate to remote peer via signaling server
        this.sendICECandidate(event.candidate);
      }
    };

    this.peerConnection.ontrack = (event) => {
      console.log('⋋ WebRTC: Remote track received');
      this.remoteStream = event.streams[0];
      this.onRemoteStreamReceived(this.remoteStream);
    };

    this.peerConnection.onconnectionstatechange = () => {
      console.log('⋋ WebRTC: Connection state:', this.peerConnection?.connectionState);
    };

    this.peerConnection.ondatachannel = (event) => {
      this.dataChannel = event.channel;
      this.setupDataChannelHandlers();
    };
  }

  /**
   * Create data channel for navigation data sync
   */
  private createDataChannel(): void {
    if (!this.peerConnection) return;

    this.dataChannel = this.peerConnection.createDataChannel('navigation-sync', {
      ordered: true,
    });

    this.setupDataChannelHandlers();
  }

  /**
   * Setup data channel event handlers
   */
  private setupDataChannelHandlers(): void {
    if (!this.dataChannel) return;

    this.dataChannel.onopen = () => {
      console.log('⋋ WebRTC: Data channel opened');
    };

    this.dataChannel.onclose = () => {
      console.log('⋋ WebRTC: Data channel closed');
    };

    this.dataChannel.onmessage = (event) => {
      console.log('⋋ WebRTC: Data received', event.data);
      this.onDataReceived(event.data);
    };

    this.dataChannel.onerror = (error) => {
      console.error('⋋ WebRTC: Data channel error', error);
    };
  }

  /**
   * Get local media stream (audio/video)
   */
  async getLocalStream(constraints?: MediaStreamConstraints): Promise<MediaStream> {
    try {
      const defaultConstraints: MediaStreamConstraints = {
        audio: this.config.enableAudio,
        video: this.config.enableVideo ? {
          width: { ideal: 1280 },
          height: { ideal: 720 },
          facingMode: 'user',
        } : false,
      };

      this.localStream = await navigator.mediaDevices.getUserMedia(
        constraints || defaultConstraints
      );

      console.log('⋋ WebRTC: Local stream acquired');
      return this.localStream;
    } catch (error) {
      console.error('⋋ WebRTC: Error getting local stream', error);
      throw error;
    }
  }

  /**
   * Get screen sharing stream
   */
  async getScreenStream(): Promise<MediaStream> {
    try {
      const stream = await navigator.mediaDevices.getDisplayMedia({
        video: {
          cursor: 'always',
        },
        audio: false,
      });

      console.log('⋋ WebRTC: Screen stream acquired');
      return stream;
    } catch (error) {
      console.error('⋋ WebRTC: Error getting screen stream', error);
      throw error;
    }
  }

  /**
   * Add local stream to peer connection
   */
  addLocalStream(stream: MediaStream): void {
    if (!this.peerConnection) return;

    stream.getTracks().forEach((track) => {
      this.peerConnection!.addTrack(track, stream);
    });

    console.log('⋋ WebRTC: Local stream added to peer connection');
  }

  /**
   * Create offer for connection
   */
  async createOffer(): Promise<RTCSessionDescriptionInit> {
    if (!this.peerConnection) throw new Error('Peer connection not initialized');

    try {
      const offer = await this.peerConnection.createOffer();
      await this.peerConnection.setLocalDescription(offer);
      
      console.log('⋋ WebRTC: Offer created');
      return offer;
    } catch (error) {
      console.error('⋋ WebRTC: Error creating offer', error);
      throw error;
    }
  }

  /**
   * Create answer for connection
   */
  async createAnswer(offer: RTCSessionDescriptionInit): Promise<RTCSessionDescriptionInit> {
    if (!this.peerConnection) throw new Error('Peer connection not initialized');

    try {
      await this.peerConnection.setRemoteDescription(new RTCSessionDescription(offer));
      const answer = await this.peerConnection.createAnswer();
      await this.peerConnection.setLocalDescription(answer);
      
      console.log('⋋ WebRTC: Answer created');
      return answer;
    } catch (error) {
      console.error('⋋ WebRTC: Error creating answer', error);
      throw error;
    }
  }

  /**
   * Set remote description
   */
  async setRemoteDescription(description: RTCSessionDescriptionInit): Promise<void> {
    if (!this.peerConnection) throw new Error('Peer connection not initialized');

    try {
      await this.peerConnection.setRemoteDescription(new RTCSessionDescription(description));
      console.log('⋋ WebRTC: Remote description set');
    } catch (error) {
      console.error('⋋ WebRTC: Error setting remote description', error);
      throw error;
    }
  }

  /**
   * Add ICE candidate
   */
  async addICECandidate(candidate: RTCIceCandidateInit): Promise<void> {
    if (!this.peerConnection) throw new Error('Peer connection not initialized');

    try {
      await this.peerConnection.addIceCandidate(new RTCIceCandidate(candidate));
      console.log('⋋ WebRTC: ICE candidate added');
    } catch (error) {
      console.error('⋋ WebRTC: Error adding ICE candidate', error);
      throw error;
    }
  }

  /**
   * Send navigation data via data channel
   */
  sendNavigationData(data: any): void {
    if (!this.dataChannel || this.dataChannel.readyState !== 'open') {
      console.warn('⋋ WebRTC: Data channel not ready');
      return;
    }

    const message = JSON.stringify({
      type: 'navigation-data',
      timestamp: Date.now(),
      data,
    });

    this.dataChannel.send(message);
    console.log('⋋ WebRTC: Navigation data sent');
  }

  /**
   * Close connection and cleanup
   */
  close(): void {
    if (this.dataChannel) {
      this.dataChannel.close();
      this.dataChannel = null;
    }

    if (this.localStream) {
      this.localStream.getTracks().forEach((track) => track.stop());
      this.localStream = null;
    }

    if (this.peerConnection) {
      this.peerConnection.close();
      this.peerConnection = null;
    }

    console.log('⋋ WebRTC Service: Connection closed');
  }

  /**
   * Get connection stats
   */
  async getStats(): Promise<RTCStatsReport | null> {
    if (!this.peerConnection) return null;

    try {
      const stats = await this.peerConnection.getStats();
      return stats;
    } catch (error) {
      console.error('⋋ WebRTC: Error getting stats', error);
      return null;
    }
  }

  // Callbacks to be overridden by application
  onRemoteStreamReceived(stream: MediaStream): void {
    console.log('⋋ WebRTC: Remote stream received callback');
  }

  onDataReceived(data: string): void {
    console.log('⋋ WebRTC: Data received callback', data);
  }

  sendICECandidate(candidate: RTCIceCandidate): void {
    console.log('⋋ WebRTC: ICE candidate to be sent via signaling', candidate);
  }
}

// Export singleton instance
export const webrtcService = new WebRTCService();

// Export for NAVΛ terminal integration
export function initializeWebRTC(): void {
  webrtcService.initialize().then(() => {
    console.log('⋋ WebRTC Service: Ready for real-time collaboration');
  });
}

