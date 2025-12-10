import React, { useState } from 'react';
import './IDEDownloadButton.css';

interface IDEDownloadButtonProps {
  className?: string;
}

export const IDEDownloadButton: React.FC<IDEDownloadButtonProps> = ({ className = '' }) => {
  const [isDownloading, setIsDownloading] = useState(false);
  const [downloadProgress, setDownloadProgress] = useState(0);

  const handleDownload = async () => {
    setIsDownloading(true);
    setDownloadProgress(0);

    try {
      // Try multiple installer locations (DMG first for macOS, then ZIP)
      const installerPaths = [
        '/NAVΛ-Studio-IDE.dmg',  // DMG installer (preferred for macOS - opens with app icon visible)
        '/NAVΛ-Studio-IDE.zip',  // ZIP installer (fallback)
        '/NAVΛ-Studio-IDE-latest.dmg',
        '/NAVΛ-Studio-IDE-complete.dmg',
        '/installers/NAVΛ-Studio-IDE.dmg',
        '/sdk/installers/NAVΛ-SDK-latest.dmg',
        '/sdk/installers/NAVΛ-SDK.dmg',
      ];
      
      let response: Response | null = null;
      let installerUrl = '';
      
      // Try each path until we find one that works
      for (const path of installerPaths) {
        try {
          const testResponse = await fetch(path, { method: 'HEAD' });
          if (testResponse.ok) {
            installerUrl = path;
            console.log('Found installer at:', path);
            break;
          }
        } catch (e) {
          // Continue to next path
          continue;
        }
      }
      
      // If no installer found, show message
      if (!installerUrl) {
        setIsDownloading(false);
        setDownloadProgress(0);
        alert(
          'NAVΛ Studio IDE installer is not available yet.\n\n' +
          'The installer is being prepared and will be available shortly.\n\n' +
          'Please try again in a few moments, or contact support if the issue persists.'
        );
        return;
      }
      
      // Download from found path
      response = await fetch(installerUrl, {
        method: 'GET',
        headers: {
          'Accept': 'application/octet-stream',
        },
      });

      if (!response.ok) {
        throw new Error(`Download failed: ${response.status} ${response.statusText}`);
      }

      const contentLength = response.headers.get('Content-Length');
      const total = contentLength ? parseInt(contentLength, 10) : 0;
      const reader = response.body?.getReader();
      const chunks: Uint8Array[] = [];
      let receivedLength = 0;

      if (!reader) {
        throw new Error('Failed to get response reader');
      }

      // Read with progress updates
      while (true) {
        const { done, value } = await reader.read();
        
        if (done) break;
        
        chunks.push(value);
        receivedLength += value.length;
        
        if (total > 0) {
          const progress = Math.round((receivedLength / total) * 100);
          setDownloadProgress(Math.min(progress, 95));
        } else {
          setDownloadProgress(Math.min(receivedLength / 1000000 * 10, 95));
        }
      }

      // Combine chunks
      const blob = new Blob(chunks as BlobPart[], { type: 'application/octet-stream' });
      
      // Verify file size
      if (blob.size === 0) {
        throw new Error('Downloaded file is empty');
      }

      // Verify minimum size (at least 1KB for ZIP, 10KB for DMG - DMG can be small for simple app bundles)
      const isZipFile = installerUrl.endsWith('.zip');
      const minSize = isZipFile ? 1024 : 10240; // 1KB for ZIP, 10KB for DMG (reasonable for app bundles)
      if (blob.size < minSize) {
        throw new Error(`Downloaded file is too small (${(blob.size / 1024).toFixed(2)}KB), may be corrupted. Minimum size: ${(minSize / 1024).toFixed(2)}KB`);
      }

      // Create download link
      const url = window.URL.createObjectURL(blob);
      const link = document.createElement('a');
      link.href = url;
      // Determine download filename based on installer type
      link.download = isZipFile ? 'NAVΛ-Studio-IDE.zip' : 'NAVΛ-Studio-IDE.dmg';
      link.style.display = 'none';
      link.style.visibility = 'hidden';
      
      // Add to DOM and trigger download
      document.body.appendChild(link);
      
      // Force download trigger
      const clickEvent = new MouseEvent('click', {
        bubbles: true,
        cancelable: true,
        view: window
      });
      
      link.dispatchEvent(clickEvent);
      
      const filename = isZipFile ? 'NAVΛ-Studio-IDE.zip' : 'NAVΛ-Studio-IDE.dmg';
      console.log(`✅ Download initiated: ${filename}`, 'Size:', (blob.size / 1024 / 1024).toFixed(2), 'MB');
      
      // Clean up after download starts
      setTimeout(() => {
        if (document.body.contains(link)) {
          document.body.removeChild(link);
        }
        window.URL.revokeObjectURL(url);
      }, 2000);

      // Complete progress
      setDownloadProgress(100);
      setIsDownloading(false);
    } catch (error) {
      console.error('Download failed:', error);
      setIsDownloading(false);
      setDownloadProgress(0);
      
      // Provide more helpful error message
      const errorMessage = error instanceof Error ? error.message : 'Unknown error';
      console.log('Download error details:', errorMessage);
      
      // Check if it's a network/CORS issue or file not found
      if (errorMessage.includes('Failed to fetch') || errorMessage.includes('NetworkError')) {
        alert('Installer not available.\n\nThe installer needs to be built first.\n\nFor developers: Run "npm run build" to create the installer.\n\nThe installer will then be available at /NAVΛ-Studio-IDE.dmg');
      } else {
        alert(`Download failed: ${errorMessage}\n\nPlease ensure the installer file exists or run "npm run build" to create it.`);
      }
    }
  };

  return (
    <button
      className={`ide-download-btn ${className} ${isDownloading ? 'downloading' : ''}`}
      onClick={handleDownload}
      disabled={isDownloading}
      title="Download NAVΛ Studio IDE"
    >
      {isDownloading ? (
        <>
          <span className="ide-btn-icon">⏳</span>
          <span className="ide-btn-text">Downloading... {downloadProgress}%</span>
        </>
      ) : (
        <>
          <span className="ide-btn-icon">⬇️</span>
          <span className="ide-btn-text">Download IDE</span>
        </>
      )}
      {isDownloading && (
        <div className="ide-progress-bar">
          <div className="ide-progress-fill" style={{ width: `${downloadProgress}%` }} />
        </div>
      )}
    </button>
  );
};

