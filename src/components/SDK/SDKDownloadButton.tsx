import React, { useState, useEffect } from 'react';
import './SDKDownloadButton.css';

interface SDKDownloadButtonProps {
  className?: string;
}

interface PlatformInfo {
  os: 'macos' | 'windows' | 'linux' | 'web';
  arch: 'x64' | 'arm64';
  name: string;
}

export const SDKDownloadButton: React.FC<SDKDownloadButtonProps> = ({ className = '' }) => {
  const [platform, setPlatform] = useState<PlatformInfo | null>(null);
  const [isDownloading, setIsDownloading] = useState(false);
  const [downloadProgress, setDownloadProgress] = useState(0);
  const [error, setError] = useState<string | null>(null);
  const [showModal, setShowModal] = useState(false);

  // Detect platform
  useEffect(() => {
    const detectPlatform = (): PlatformInfo => {
      const userAgent = navigator.userAgent.toLowerCase();
      const platform = navigator.platform.toLowerCase();

      if (platform.includes('mac') || userAgent.includes('mac')) {
        return {
          os: 'macos',
          arch: navigator.userAgent.includes('arm') || platform.includes('arm') ? 'arm64' : 'x64',
          name: 'macOS',
        };
      } else if (userAgent.includes('win')) {
        return {
          os: 'windows',
          arch: 'x64',
          name: 'Windows',
        };
      } else if (userAgent.includes('linux')) {
        return {
          os: 'linux',
          arch: 'x64',
          name: 'Linux',
        };
      } else {
        return {
          os: 'web',
          arch: 'x64',
          name: 'Web',
        };
      }
    };

    setPlatform(detectPlatform());
  }, []);

  const handleSDKDownload = async () => {
    if (!platform) return;

    setIsDownloading(true);
    setDownloadProgress(0);
    setError(null);

    try {
      // For web, open npm page
      if (platform.os === 'web') {
        window.open('https://www.npmjs.com/package/@nava/sdk-web', '_blank');
        setIsDownloading(false);
        setShowModal(true);
        return;
      }

      // For native platforms, try to download installer or show build instructions
      const installerUrl = getInstallerUrl(platform);
      const filename = getInstallerFilename(platform);
      
      if (installerUrl) {
        // Download the installer with integrity checks
        try {
          const response = await fetch(installerUrl, {
            method: 'GET',
            headers: {
              'Accept': 'application/octet-stream',
            },
          });

          if (!response.ok) {
            // File doesn't exist (404) or server error
            console.log('Installer not found at:', installerUrl, 'Status:', response.status);
            if (response.status === 404) {
              setError(`Installer not available. The installer should be built during the app build process.\n\nIf you're a developer, run: npm run build\n\nOtherwise, please contact support.`);
            } else {
              setError(`Download failed: ${response.status} ${response.statusText}`);
            }
            setIsDownloading(false);
            setDownloadProgress(0);
            setShowModal(true);
            return;
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
          const blob = new Blob(chunks, { type: 'application/octet-stream' });
          
          // Verify file size
          if (blob.size === 0) {
            throw new Error('Downloaded file is empty');
          }

          // Verify minimum size (at least 1KB)
          if (blob.size < 1024) {
            throw new Error('Downloaded file is too small, may be corrupted');
          }

          // Create download link with proper attributes
          const url = window.URL.createObjectURL(blob);
          const link = document.createElement('a');
          link.href = url;
          link.download = filename;
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
          
          console.log('✅ Download initiated:', filename, 'Size:', (blob.size / 1024 / 1024).toFixed(2), 'MB');
          
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
          
          // Show success message
          setTimeout(() => {
            setShowModal(true);
          }, 500);
        } catch (downloadError) {
          console.error('Download failed:', downloadError);
          const errorMessage = downloadError instanceof Error ? downloadError.message : 'Download failed';
          
          // If it's a network error, suggest building the installer
          if (errorMessage.includes('Failed to fetch') || errorMessage.includes('NetworkError')) {
            setError(`Installer not available. Please build it first:\n\ncd sdk\n./scripts/build-all.sh\n\nThis will create the DMG file.`);
          } else {
            setError(errorMessage);
          }
          
          setIsDownloading(false);
          setDownloadProgress(0);
          setShowModal(true);
        }
      } else {
        // No installer available, show build instructions
        setIsDownloading(false);
        setShowModal(true);
      }
    } catch (error) {
      console.error('Download failed:', error);
      setError(error instanceof Error ? error.message : 'Download failed');
      setIsDownloading(false);
      setDownloadProgress(0);
      
      // Show error in modal
      setTimeout(() => {
        setShowModal(true);
      }, 500);
    }
  };

  const getInstallerUrl = (platform: PlatformInfo): string => {
    // Use relative paths to the installers directory in public folder
    const baseUrl = '/sdk/installers';
    
    switch (platform.os) {
      case 'macos':
        return `${baseUrl}/NAVΛ-SDK-latest.dmg`;
      case 'windows':
        return `${baseUrl}/NAVΛ-SDK-Setup-latest.exe`;
      case 'linux':
        return `${baseUrl}/nava-sdk_1.0.0_${platform.arch}.deb`;
      default:
        return '';
    }
  };

  const getInstallerFilename = (platform: PlatformInfo): string => {
    switch (platform.os) {
      case 'macos':
        return 'NAVΛ-SDK.dmg';
      case 'windows':
        return 'NAVΛ-SDK-Setup.exe';
      case 'linux':
        return `nava-sdk_1.0.0_${platform.arch}.deb`;
      default:
        return 'nava-sdk.zip';
    }
  };

  const getInstallSteps = (platform: PlatformInfo) => {
    switch (platform.os) {
      case 'macos':
        return (
          <ol className="sdk-steps-list">
            <li>Open the downloaded <code>.dmg</code> file</li>
            <li>Drag <strong>NAVΛ SDK.app</strong> to your <strong>Applications</strong> folder</li>
            <li>Open Applications and launch <strong>NAVΛ SDK</strong></li>
            <li>The SDK will be available in your Applications folder</li>
          </ol>
        );
      case 'windows':
        return (
          <ol className="sdk-steps-list">
            <li>Run the downloaded <code>.exe</code> installer</li>
            <li>Follow the installation wizard</li>
            <li>Choose installation directory (default: Program Files\NAVA SDK)</li>
            <li>Click <strong>Install</strong> to complete</li>
            <li>Launch NAVΛ SDK from Start Menu</li>
          </ol>
        );
      case 'linux':
        return (
          <ol className="sdk-steps-list">
            <li>Open terminal in the directory with the downloaded <code>.deb</code> file</li>
            <li>Run: <code>sudo dpkg -i nava-sdk_1.0.0_*.deb</code></li>
            <li>If dependencies are missing, run: <code>sudo apt-get install -f</code></li>
            <li>Launch from Applications menu or run: <code>nava-sdk</code></li>
          </ol>
        );
      default:
        return null;
    }
  };

  const getInstallInstructions = () => {
    if (!platform) return '';

    switch (platform.os) {
      case 'macos':
        return `
1. The SDK is included in NAVΛ Studio IDE
2. Open Terminal in the IDE or your system terminal
3. Navigate to SDK directory: cd sdk/native
4. Build the SDK: cargo build --release
5. For Python: cd ../bindings/python && pip install -e .
6. For Node.js: cd ../bindings/nodejs && npm install
        `;
      case 'windows':
        return `
1. The SDK is included in NAVΛ Studio IDE
2. Open Command Prompt or PowerShell
3. Navigate to SDK directory: cd sdk\\native
4. Build the SDK: cargo build --release
5. For Python: cd ..\\bindings\\python && pip install -e .
6. For Node.js: cd ..\\bindings\\nodejs && npm install
        `;
      case 'linux':
        return `
1. The SDK is included in NAVΛ Studio IDE
2. Open Terminal
3. Navigate to SDK directory: cd sdk/native
4. Build the SDK: cargo build --release
5. For Python: cd ../bindings/python && pip install -e .
6. For Node.js: cd ../bindings/nodejs && npm install
        `;
      case 'web':
        return `
1. Install via npm: npm install @nava/sdk-web
2. Or build from source: cd sdk/web && npm install && npm run build
3. Import in your code: import { NavigationField } from '@nava/sdk-web'
4. Or use via CDN (when available): <script src="https://cdn.nava.studio/sdk/v1/nava-sdk.js"></script>
        `;
      default:
        return '';
    }
  };

  if (!platform) {
    return null;
  }

  return (
    <>
      <button
        className={`sdk-download-btn ${className} ${isDownloading ? 'downloading' : ''}`}
        onClick={handleSDKDownload}
        disabled={isDownloading}
        title={`Download NAVΛ SDK for ${platform.name}`}
      >
        {isDownloading ? (
          <>
            <span className="sdk-btn-icon">⏳</span>
            <span className="sdk-btn-text">Downloading... {downloadProgress}%</span>
          </>
        ) : (
          <>
            <span className="sdk-btn-logo">
              <span className="sdk-logo-lambda">λ</span>
              <span className="sdk-logo-capital">Λ</span>
            </span>
            <span className="sdk-btn-text">NAVΛ SDK</span>
            <span className="sdk-btn-platform">{platform.name}</span>
          </>
        )}
        {isDownloading && (
          <div className="sdk-progress-bar">
            <div className="sdk-progress-fill" style={{ width: `${downloadProgress}%` }} />
          </div>
        )}
      </button>

      {/* Installation Modal */}
      {showModal && (
        <div className="sdk-modal-overlay" onClick={() => setShowModal(false)}>
          <div className="sdk-modal" onClick={(e) => e.stopPropagation()}>
            <div className="sdk-modal-header">
              <div className="sdk-modal-header-content">
                <div className="sdk-modal-logo-container">
                  <span className="sdk-modal-logo-lambda">λ</span>
                  <span className="sdk-modal-logo-capital">Λ</span>
                </div>
                <div>
                  <h2>NAVΛ SDK Installation</h2>
                  <p className="sdk-platform-badge">Detected: {platform.name} {platform.arch}</p>
                </div>
              </div>
              <button className="sdk-modal-close" onClick={() => setShowModal(false)}>
                ✕
              </button>
            </div>
            <div className="sdk-modal-content">
              {error ? (
                <div className="sdk-error-message">
                  <span className="sdk-error-icon">⚠️</span>
                  <div>
                    <p><strong>Installer Not Available</strong></p>
                    <p style={{ whiteSpace: 'pre-line', fontFamily: 'monospace', fontSize: '13px', lineHeight: '1.6' }}>{error}</p>
                    <p style={{ fontSize: '12px', marginTop: '12px', opacity: 0.8 }}>
                      The installer needs to be built first. Follow the build instructions below.
                    </p>
                  </div>
                </div>
              ) : downloadProgress === 100 ? (
                <div className="sdk-success-message">
                  <span className="sdk-success-icon">✅</span>
                  <p>NAVΛ SDK installer downloaded successfully!</p>
                  <p style={{ fontSize: '12px', marginTop: '8px', opacity: 0.8 }}>
                    Check your Downloads folder for: <strong>{getInstallerFilename(platform)}</strong>
                  </p>
                </div>
              ) : (
                <div className="sdk-success-message">
                  <span className="sdk-success-icon">📦</span>
                  <p>NAVΛ SDK installer ready for {platform.name}!</p>
                </div>
              )}
              
              {!error && (
                <div className="sdk-download-section">
                  <h3>📥 Installation Instructions</h3>
                  <p style={{ marginBottom: '16px', color: 'rgba(255, 255, 255, 0.7)', fontSize: '14px' }}>
                    After downloading, follow these steps to install:
                  </p>
                  <div className="sdk-install-steps">
                    {getInstallSteps(platform)}
                  </div>
                </div>
              )}
              <div className="sdk-modal-footer">
                <a
                  href="/sdk/README.md"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="sdk-docs-link"
                >
                  📚 View Full Documentation
                </a>
                <button className="sdk-modal-btn" onClick={() => setShowModal(false)}>
                  Got it!
                </button>
              </div>
            </div>
          </div>
        </div>
      )}

      {/* Error handling - silent retry, no visible error UI */}
    </>
  );
};

