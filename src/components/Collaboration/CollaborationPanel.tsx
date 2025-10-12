import React, { useState, useEffect } from 'react';
import { Users, Video, Mic, Share2 } from 'lucide-react';
import { collaborationService } from '../../services/collaboration-service';
import './CollaborationPanel.css';

export const CollaborationPanel: React.FC = () => {
  const [isCollaborating, setIsCollaborating] = useState(false);
  const [roomId, setRoomId] = useState('');
  const [peers, setPeers] = useState<any[]>([]);

  useEffect(() => {
    collaborationService.initialize('Frank Van Laarhoven');
  }, []);

  const startCollaboration = async () => {
    const newRoomId = await collaborationService.createRoom('vnc-' + Date.now());
    setRoomId(newRoomId);
    setIsCollaborating(true);
  };

  const joinCollaboration = async () => {
    await collaborationService.joinRoom(roomId);
    setIsCollaborating(true);
  };

  const shareRoom = () => {
    navigator.clipboard.writeText(`http://localhost:3000?room=${roomId}`);
    alert('Room link copied to clipboard!');
  };

  return (
    <div className="collaboration-panel">
      <div className="collab-header">
        <Users size={20} />
        <h3>Collaboration</h3>
      </div>

      {!isCollaborating ? (
        <div className="collab-setup">
          <button className="collab-btn primary" onClick={startCollaboration}>
            <Users size={16} />
            Start Collaboration Session
          </button>

          <div className="collab-divider">or</div>

          <input
            type="text"
            placeholder="Enter room ID"
            value={roomId}
            onChange={(e) => setRoomId(e.target.value)}
            className="collab-input"
          />
          <button
            className="collab-btn secondary"
            onClick={joinCollaboration}
            disabled={!roomId}
          >
            Join Room
          </button>
        </div>
      ) : (
        <div className="collab-active">
          <div className="collab-room-info">
            <strong>Room:</strong> {roomId}
            <button className="share-btn" onClick={shareRoom}>
              <Share2 size={14} />
            </button>
          </div>

          <div className="collab-peers">
            <h4>Active Collaborators ({peers.length + 1})</h4>
            <div className="peer-list">
              <div className="peer-item you">
                <div className="peer-avatar" style={{ background: '#00ff00' }}>
                  F
                </div>
                <span>You</span>
              </div>
              {peers.map((peer) => (
                <div key={peer.id} className="peer-item">
                  <div className="peer-avatar" style={{ background: peer.color }}>
                    {peer.name[0]}
                  </div>
                  <span>{peer.name}</span>
                </div>
              ))}
            </div>
          </div>

          <div className="collab-controls">
            <button className="control-btn">
              <Video size={16} />
            </button>
            <button className="control-btn">
              <Mic size={16} />
            </button>
            <button className="control-btn danger" onClick={() => setIsCollaborating(false)}>
              Leave
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

