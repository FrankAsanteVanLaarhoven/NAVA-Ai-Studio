/**
 * Scenario Panel Component
 * 
 * Allows testing multiple NAVA code variants with different parameters
 */

import React, { useState, useEffect } from 'react';
import { navaPreviewEngine } from '../../services/nava-preview-engine';
import { navaRuntimeService } from '../../services/nava-runtime-service';
import './ScenarioPanel.css';

interface Scenario {
  id: string;
  name: string;
  manifold: string;
  start: { x: number; y: number; z?: number };
  goal: { x: number; y: number; z?: number };
  obstacles: any[];
  costModel: string;
  navaCode: string;
  result?: any;
  status?: 'pending' | 'running' | 'success' | 'error';
}

interface ScenarioPanelProps {
  baseCode: string;
  onScenarioSelect?: (scenario: Scenario) => void;
}

export const ScenarioPanel: React.FC<ScenarioPanelProps> = ({
  baseCode,
  onScenarioSelect,
}) => {
  const [scenarios, setScenarios] = useState<Scenario[]>([]);
  const [selectedScenario, setSelectedScenario] = useState<string | null>(null);
  const [numVariants, setNumVariants] = useState(3);
  const [isGenerating, setIsGenerating] = useState(false);
  const [isRunning, setIsRunning] = useState(false);

  // Generate scenarios from base code
  const generateScenarios = () => {
    setIsGenerating(true);
    try {
      const generated = navaPreviewEngine.generateScenarios(baseCode, numVariants);
      setScenarios(generated);
      if (generated.length > 0) {
        setSelectedScenario(generated[0].id);
      }
    } catch (error) {
      console.error('Error generating scenarios:', error);
    } finally {
      setIsGenerating(false);
    }
  };

  // Run a scenario
  const runScenario = async (scenario: Scenario) => {
    scenario.status = 'running';
    setScenarios([...scenarios]);

    try {
      const result = await navaRuntimeService.runNAVA(scenario.navaCode, {
        preview: true,
      });

      scenario.result = result;
      scenario.status = result.success ? 'success' : 'error';
    } catch (error: any) {
      scenario.status = 'error';
      scenario.result = { error: error.message };
    } finally {
      setScenarios([...scenarios]);
    }
  };

  // Run all scenarios
  const runAllScenarios = async () => {
    setIsRunning(true);
    for (const scenario of scenarios) {
      await runScenario(scenario);
    }
    setIsRunning(false);
  };

  // Auto-generate on base code change
  useEffect(() => {
    if (baseCode) {
      generateScenarios();
    }
  }, [baseCode]);

  return (
    <div className="scenario-panel">
      <div className="scenario-panel-header">
        <h3>Scenario Variants</h3>
        <div className="scenario-controls">
          <label>
            Variants:
            <input
              type="number"
              min="1"
              max="10"
              value={numVariants}
              onChange={(e) => setNumVariants(parseInt(e.target.value) || 3)}
            />
          </label>
          <button onClick={generateScenarios} disabled={isGenerating}>
            {isGenerating ? 'Generating...' : 'Generate'}
          </button>
          {scenarios.length > 0 && (
            <button onClick={runAllScenarios} disabled={isRunning}>
              {isRunning ? 'Running...' : 'Run All'}
            </button>
          )}
        </div>
      </div>

      <div className="scenario-list">
        {scenarios.map((scenario) => (
          <div
            key={scenario.id}
            className={`scenario-item ${selectedScenario === scenario.id ? 'selected' : ''} ${scenario.status || ''}`}
            onClick={() => {
              setSelectedScenario(scenario.id);
              onScenarioSelect?.(scenario);
            }}
          >
            <div className="scenario-header">
              <span className="scenario-name">{scenario.name}</span>
              <span className={`scenario-status ${scenario.status || 'pending'}`}>
                {scenario.status || 'Pending'}
              </span>
            </div>
            <div className="scenario-details">
              <div className="scenario-param">
                <span className="param-label">Start:</span>
                <span className="param-value">
                  ({scenario.start.x.toFixed(1)}, {scenario.start.y.toFixed(1)}
                  {scenario.start.z !== undefined ? `, ${scenario.start.z.toFixed(1)}` : ''})
                </span>
              </div>
              <div className="scenario-param">
                <span className="param-label">Goal:</span>
                <span className="param-value">
                  ({scenario.goal.x.toFixed(1)}, {scenario.goal.y.toFixed(1)}
                  {scenario.goal.z !== undefined ? `, ${scenario.goal.z.toFixed(1)}` : ''})
                </span>
              </div>
              <div className="scenario-param">
                <span className="param-label">Manifold:</span>
                <span className="param-value">{scenario.manifold}</span>
              </div>
              <div className="scenario-param">
                <span className="param-label">Cost:</span>
                <span className="param-value">{scenario.costModel}</span>
              </div>
            </div>
            {scenario.result && (
              <div className="scenario-result">
                {scenario.result.success ? (
                  <div className="result-success">
                    <span>✓ Path length: {scenario.result.metadata?.pathLength?.toFixed(2) || 'N/A'}</span>
                    {scenario.result.metadata?.executionTime && (
                      <span>Time: {scenario.result.metadata.executionTime.toFixed(2)}ms</span>
                    )}
                  </div>
                ) : (
                  <div className="result-error">
                    <span>✗ {scenario.result.error || 'Execution failed'}</span>
                  </div>
                )}
              </div>
            )}
            <button
              className="scenario-run-btn"
              onClick={(e) => {
                e.stopPropagation();
                runScenario(scenario);
              }}
              disabled={scenario.status === 'running'}
            >
              {scenario.status === 'running' ? 'Running...' : 'Run'}
            </button>
          </div>
        ))}
      </div>

      {scenarios.length === 0 && (
        <div className="scenario-empty">
          <p>No scenarios generated yet.</p>
          <p>Click "Generate" to create variants from the base code.</p>
        </div>
      )}
    </div>
  );
};

