/**
 * Learning Mode Component
 * 
 * Provides educational explanations at multiple levels (GCSE → PhD)
 */

import React, { useState, useEffect } from 'react';
import { navaExplanationGenerator } from '../../services/nava-explanation-service';
import './LearningMode.css';

interface LearningModeProps {
  code: string;
  level?: 'gcse' | 'alevel' | 'undergrad' | 'phd';
  focus?: string;
  onLevelChange?: (level: 'gcse' | 'alevel' | 'undergrad' | 'phd') => void;
}

export const LearningMode: React.FC<LearningModeProps> = ({
  code,
  level: initialLevel = 'alevel',
  focus,
  onLevelChange,
}) => {
  const [level, setLevel] = useState<'gcse' | 'alevel' | 'undergrad' | 'phd'>(initialLevel);
  const [explanation, setExplanation] = useState<string>('');
  const [isGenerating, setIsGenerating] = useState(false);
  const [lineExplanations, setLineExplanations] = useState<Map<number, string>>(new Map());

  useEffect(() => {
    generateExplanation();
  }, [code, level, focus]);

  const generateExplanation = async () => {
    if (!code.trim()) return;

    setIsGenerating(true);
    try {
      const expl = await navaExplanationGenerator.generateExplanation(code, level, focus);
      setExplanation(expl);

      // Generate line-by-line explanations
      const lines = code.split('\n');
      const lineExpls = new Map<number, string>();
      
      for (let i = 0; i < lines.length; i++) {
        const line = lines[i].trim();
        if (line && !line.startsWith('//')) {
          const lineExpl = await navaExplanationGenerator.explainLine(line, level);
          if (lineExpl) {
            lineExpls.set(i + 1, lineExpl);
          }
        }
      }
      
      setLineExplanations(lineExpls);
    } catch (error) {
      console.error('Error generating explanation:', error);
      setExplanation('Error generating explanation. Please try again.');
    } finally {
      setIsGenerating(false);
    }
  };

  const handleLevelChange = (newLevel: 'gcse' | 'alevel' | 'undergrad' | 'phd') => {
    setLevel(newLevel);
    onLevelChange?.(newLevel);
  };

  const levels = [
    { id: 'gcse', label: 'GCSE', description: 'Simple, accessible explanations' },
    { id: 'alevel', label: 'A-Level', description: 'Mathematical concepts with geometry' },
    { id: 'undergrad', label: 'Undergrad', description: 'Manifold theory and optimization' },
    { id: 'phd', label: 'PhD', description: 'Advanced mathematical formalism' },
  ];

  return (
    <div className="learning-mode">
      <div className="learning-header">
        <h3>Learning Mode</h3>
        <div className="level-selector">
          {levels.map((lvl) => (
            <button
              key={lvl.id}
              className={`level-btn ${level === lvl.id ? 'active' : ''}`}
              onClick={() => handleLevelChange(lvl.id as any)}
              title={lvl.description}
            >
              {lvl.label}
            </button>
          ))}
        </div>
      </div>

      <div className="learning-content">
        {isGenerating ? (
          <div className="loading-state">
            <div className="spinner"></div>
            <p>Generating explanation...</p>
          </div>
        ) : (
          <>
            <div className="overview-explanation">
              <h4>Overview</h4>
              <div className="explanation-text">
                {explanation.split('\n\n').map((para, i) => (
                  <p key={i}>{para}</p>
                ))}
              </div>
            </div>

            <div className="line-by-line">
              <h4>Line-by-Line Explanation</h4>
              <div className="code-with-explanations">
                {code.split('\n').map((line, index) => {
                  const lineNum = index + 1;
                  const lineExpl = lineExplanations.get(lineNum);
                  const isCommented = line.trim().startsWith('//');

                  return (
                    <div key={index} className="code-line">
                      <div className="line-number">{lineNum}</div>
                      <div className="line-content">
                        <code className={isCommented ? 'comment' : ''}>{line || ' '}</code>
                        {lineExpl && (
                          <div className="line-explanation">
                            <span className="explanation-icon">💡</span>
                            <span className="explanation-text">{lineExpl}</span>
                          </div>
                        )}
                      </div>
                    </div>
                  );
                })}
              </div>
            </div>

            {focus && (
              <div className="focus-section">
                <h4>Focus Area</h4>
                <p>{focus}</p>
              </div>
            )}
          </>
        )}
      </div>

      <div className="learning-footer">
        <button onClick={generateExplanation} disabled={isGenerating}>
          {isGenerating ? 'Generating...' : 'Regenerate Explanation'}
        </button>
      </div>
    </div>
  );
};

