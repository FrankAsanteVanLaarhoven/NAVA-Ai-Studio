import React, { useState, useEffect, useRef } from 'react';
import { getDownloads, removeDownload, clearDownloads, getFileIcon, type DownloadItem } from '../../utils/downloadStorage';
import './DownloadsStack.css';

interface DownloadsStackProps {
  onClose: () => void;
}

export const DownloadsStack: React.FC<DownloadsStackProps> = ({ onClose }) => {
  const [downloads, setDownloads] = useState<DownloadItem[]>([]);
  const [hoveredItem, setHoveredItem] = useState<string | null>(null);
  const stackRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    // Load downloads
    setDownloads(getDownloads());

    // Listen for new downloads
    const handleDownload = (event: CustomEvent) => {
      setDownloads(getDownloads());
    };

    window.addEventListener('nava:download-complete', handleDownload as EventListener);
    return () => {
      window.removeEventListener('nava:download-complete', handleDownload as EventListener);
    };
  }, []);

  // Close on click outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (stackRef.current && !stackRef.current.contains(event.target as Node)) {
        onClose();
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [onClose]);

  const handleOpen = (download: DownloadItem) => {
    // Try to open the file
    if (download.url) {
      window.open(download.url, '_blank');
    } else if (download.path) {
      // Try to open local file path
      console.log('[DownloadsStack] Opening file:', download.path);
      // In a real implementation, this would use the File System Access API
      // or trigger a file open dialog
    }
  };

  const handleRevealInFinder = (download: DownloadItem) => {
    // Show file in finder/explorer
    console.log('[DownloadsStack] Revealing in finder:', download.path);
    // Navigate to explorer and highlight the file
    const event = new CustomEvent('nava:highlight-file', { detail: { path: download.path } });
    window.dispatchEvent(event);
    window.location.href = '/app.html?activity=explorer';
  };

  const handleRemove = (downloadId: string, e: React.MouseEvent) => {
    e.stopPropagation();
    if (removeDownload(downloadId)) {
      setDownloads(getDownloads());
    }
  };

  const handleClearAll = () => {
    if (confirm('Clear all downloads?')) {
      clearDownloads();
      setDownloads([]);
    }
  };

  const formatFileSize = (bytes: number): string => {
    if (bytes === 0) return '0 B';
    const k = 1024;
    const sizes = ['B', 'KB', 'MB', 'GB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return Math.round((bytes / Math.pow(k, i)) * 100) / 100 + ' ' + sizes[i];
  };

  const formatDate = (timestamp: number): string => {
    const date = new Date(timestamp);
    const now = new Date();
    const diff = now.getTime() - date.getTime();
    const days = Math.floor(diff / (1000 * 60 * 60 * 24));

    if (days === 0) {
      return 'Today';
    } else if (days === 1) {
      return 'Yesterday';
    } else if (days < 7) {
      return `${days} days ago`;
    } else {
      return date.toLocaleDateString();
    }
  };

  return (
    <div ref={stackRef} className="downloads-stack-container">
      <div className="downloads-stack-header">
        <h3>Downloads</h3>
        {downloads.length > 0 && (
          <button className="clear-all-btn" onClick={handleClearAll} title="Clear All">
            Clear All
          </button>
        )}
      </div>

      <div className="downloads-stack-content">
        {downloads.length === 0 ? (
          <div className="downloads-empty">
            <div className="downloads-empty-icon">📥</div>
            <p>No downloads yet</p>
          </div>
        ) : (
          <div className="downloads-list">
            {downloads.map((download) => (
              <div
                key={download.id}
                className={`download-item ${hoveredItem === download.id ? 'hovered' : ''}`}
                onMouseEnter={() => setHoveredItem(download.id)}
                onMouseLeave={() => setHoveredItem(null)}
                onClick={() => handleOpen(download)}
              >
                <div className="download-icon">
                  {download.icon || getFileIcon(download.type, download.name)}
                </div>
                <div className="download-info">
                  <div className="download-name" title={download.name}>
                    {download.name}
                  </div>
                  <div className="download-meta">
                    <span>{formatFileSize(download.size)}</span>
                    <span>•</span>
                    <span>{formatDate(download.downloadedAt)}</span>
                  </div>
                </div>
                <div className="download-actions">
                  {hoveredItem === download.id && (
                    <>
                      <button
                        className="download-action-btn"
                        onClick={(e) => {
                          e.stopPropagation();
                          handleRevealInFinder(download);
                        }}
                        title="Reveal in Finder"
                      >
                        🔍
                      </button>
                      <button
                        className="download-action-btn"
                        onClick={(e) => handleRemove(download.id, e)}
                        title="Remove"
                      >
                        ✕
                      </button>
                    </>
                  )}
                </div>
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
};

