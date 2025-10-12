import React from 'react';

interface ResultsPanelProps {
  results: any[];
  clearResults: () => void;
}

export const ResultsPanel: React.FC<ResultsPanelProps> = ({ results, clearResults }) => {
  return (
    <div className="results-panel">
      <div className="results-header">
        <h3>Execution Results</h3>
        <button onClick={clearResults}>Clear</button>
      </div>
      <div className="results-content">
        {results.length === 0 ? (
          <div className="no-results">No results yet. Run your code to see output.</div>
        ) : (
          results.map((result, index) => (
            <div key={index} className="result-item">
              <span className="result-index">{index + 1}.</span>
              <pre className="result-value">{JSON.stringify(result, null, 2)}</pre>
            </div>
          ))
        )}
      </div>
    </div>
  );
};

