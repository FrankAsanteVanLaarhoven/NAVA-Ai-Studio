import React, { useState, useEffect } from 'react';
import { lspClient } from '../../services/lsp-client';

interface LivePreviewProps {
  code: string;
  autoRun: boolean;
}

export const LivePreview: React.FC<LivePreviewProps> = ({ code, autoRun }) => {
  const [output, setOutput] = useState<string>('');
  const [isRunning, setIsRunning] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (autoRun && code.trim()) {
      runPreview();
    }
  }, [code, autoRun]);

  const runPreview = async () => {
    setIsRunning(true);
    setError(null);

    try {
      const result = await lspClient.runLivePreview(code);
      setOutput(result);
    } catch (err) {
      setError(String(err));
    } finally {
      setIsRunning(false);
    }
  };

  return (
    <div className="live-preview-panel">
      <div className="preview-header">
        <h3>Live Preview</h3>
        <button onClick={runPreview} disabled={isRunning}>
          {isRunning ? 'Running...' : 'Run'}
        </button>
      </div>
      <div className="preview-output">
        {error ? (
          <div className="error">{error}</div>
        ) : (
          <pre>{output || 'Output will appear here...'}</pre>
        )}
      </div>
    </div>
  );
};

