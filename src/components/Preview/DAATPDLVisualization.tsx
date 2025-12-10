/**
 * DAAT/PDL/RT-shields Visualization Component
 * 
 * Visualizes timing contracts, tier switches, and real-time safety metrics
 */

import React, { useState, useEffect, useRef } from 'react';
import { TimingProfile } from '../../services/nava-runtime-service';
import { navaPreviewEngine } from '../../services/nava-preview-engine';
import './DAATPDLVisualization.css';

interface DAATPDLVisualizationProps {
  path: Array<{ x: number; y: number; z?: number; t?: number }>;
  timingProfile?: TimingProfile;
  frequency?: number; // Hz
  dmrThreshold?: number;
  ajThreshold?: number;
  ttpThreshold?: number;
  ttbThreshold?: number;
}

export const DAATPDLVisualization: React.FC<DAATPDLVisualizationProps> = ({
  path,
  timingProfile,
  frequency = 50,
  dmrThreshold = 0.005,
  ajThreshold = 10,
  ttpThreshold = 10,
  ttbThreshold = 100,
}) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [selectedSegment, setSelectedSegment] = useState<number | null>(null);
  const [timePosition, setTimePosition] = useState(0);

  // Generate timing profile if not provided
  const profile = timingProfile || navaPreviewEngine.generateTimingProfile(path, {
    frequency,
    dmrThreshold,
    ajThreshold,
    ttpThreshold,
  });

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Set canvas size
    canvas.width = canvas.offsetWidth;
    canvas.height = canvas.offsetHeight;

    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw timing profile
    drawTimingProfile(ctx, canvas.width, canvas.height);
  }, [profile, selectedSegment, timePosition]);

  const drawTimingProfile = (
    ctx: CanvasRenderingContext2D,
    width: number,
    height: number
  ) => {
    const padding = 40;
    const chartWidth = width - padding * 2;
    const chartHeight = height - padding * 2;
    const segmentWidth = chartWidth / profile.segments.length;

    // Draw grid
    ctx.strokeStyle = '#3e3e3e';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 10; i++) {
      const y = padding + (chartHeight / 10) * i;
      ctx.beginPath();
      ctx.moveTo(padding, y);
      ctx.lineTo(width - padding, y);
      ctx.stroke();
    }

    // Draw segments
    profile.segments.forEach((segment, index) => {
      const x = padding + segmentWidth * index;
      const segmentHeight = chartHeight;

      // Color based on tier
      let color = '#6b7280'; // Default
      if (segment.tier === 'tier-0') {
        color = '#ef4444'; // Red - Emergency
      } else if (segment.tier === 'tier-1') {
        color = '#ffa500'; // Orange - Chunked VLA
      } else if (segment.tier === 'tier-2') {
        color = '#22c55e'; // Green - Offline planner
      }

      // Color based on contract status
      if (segment.contractStatus === 'fail') {
        color = '#ef4444';
      } else if (segment.contractStatus === 'warning') {
        color = '#ffa500';
      } else if (segment.contractStatus === 'pass') {
        color = '#22c55e';
      }

      // Draw segment bar
      ctx.fillStyle = selectedSegment === index ? color : color + '80';
      ctx.fillRect(x, padding, segmentWidth, segmentHeight);

      // Draw border
      ctx.strokeStyle = selectedSegment === index ? '#fff' : color;
      ctx.lineWidth = selectedSegment === index ? 2 : 1;
      ctx.strokeRect(x, padding, segmentWidth, segmentHeight);

      // Draw metrics text
      ctx.fillStyle = '#fff';
      ctx.font = '10px monospace';
      ctx.textAlign = 'center';
      ctx.fillText(
        `DMR: ${(segment.dmr * 100).toFixed(1)}%`,
        x + segmentWidth / 2,
        padding + 15
      );
      ctx.fillText(
        `AJ: ${segment.aj.toFixed(1)}ms`,
        x + segmentWidth / 2,
        padding + 28
      );
      ctx.fillText(
        `TTP: ${segment.ttp.toFixed(1)}ms`,
        x + segmentWidth / 2,
        padding + 41
      );
    });

    // Draw time position indicator
    if (timePosition > 0) {
      const timeX = padding + (chartWidth * timePosition) / 100;
      ctx.strokeStyle = '#3b82f6';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(timeX, padding);
      ctx.lineTo(timeX, height - padding);
      ctx.stroke();
    }

    // Draw legend
    drawLegend(ctx, width - padding - 150, padding);
  };

  const drawLegend = (ctx: CanvasRenderingContext2D, x: number, y: number) => {
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'left';

    const items = [
      { label: 'Tier-0 (Reflex)', color: '#ef4444' },
      { label: 'Tier-1 (VLA)', color: '#ffa500' },
      { label: 'Tier-2 (Planner)', color: '#22c55e' },
      { label: 'Contract Pass', color: '#22c55e' },
      { label: 'Contract Warning', color: '#ffa500' },
      { label: 'Contract Fail', color: '#ef4444' },
    ];

    items.forEach((item, index) => {
      ctx.fillStyle = item.color;
      ctx.fillRect(x, y + index * 18, 12, 12);
      ctx.fillStyle = '#d4d4d4';
      ctx.fillText(item.label, x + 18, y + index * 18 + 10);
    });
  };

  const handleCanvasClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const padding = 40;
    const chartWidth = canvas.width - padding * 2;
    const segmentWidth = chartWidth / profile.segments.length;

    const segmentIndex = Math.floor((x - padding) / segmentWidth);
    if (segmentIndex >= 0 && segmentIndex < profile.segments.length) {
      setSelectedSegment(segmentIndex);
    }
  };

  const selectedSegmentData = selectedSegment !== null ? profile.segments[selectedSegment] : null;

  return (
    <div className="daat-pdl-visualization">
      <div className="visualization-header">
        <h3>DAAT/PDL/RT-shields Timing Profile</h3>
        <div className="timing-controls">
          <label>
            Frequency:
            <input
              type="number"
              value={frequency}
              readOnly
              disabled
            />
            Hz
          </label>
          <label>
            Time Position:
            <input
              type="range"
              min="0"
              max="100"
              value={timePosition}
              onChange={(e) => setTimePosition(parseInt(e.target.value))}
            />
            {timePosition}%
          </label>
        </div>
      </div>

      <div className="visualization-canvas-container">
        <canvas
          ref={canvasRef}
          className="timing-canvas"
          onClick={handleCanvasClick}
        />
      </div>

      {selectedSegmentData && (
        <div className="segment-details">
          <h4>Segment {selectedSegment! + 1} Details</h4>
          <div className="details-grid">
            <div className="detail-item">
              <span className="detail-label">Tier:</span>
              <span className={`detail-value tier-${selectedSegmentData.tier}`}>
                {selectedSegmentData.tier}
              </span>
            </div>
            <div className="detail-item">
              <span className="detail-label">DMR:</span>
              <span className={`detail-value ${selectedSegmentData.dmr <= dmrThreshold ? 'pass' : 'fail'}`}>
                {(selectedSegmentData.dmr * 100).toFixed(2)}%
                {selectedSegmentData.dmr <= dmrThreshold ? ' ✓' : ' ✗'}
              </span>
            </div>
            <div className="detail-item">
              <span className="detail-label">Arrival Jitter:</span>
              <span className={`detail-value ${selectedSegmentData.aj <= ajThreshold ? 'pass' : 'fail'}`}>
                {selectedSegmentData.aj.toFixed(2)}ms
                {selectedSegmentData.aj <= ajThreshold ? ' ✓' : ' ✗'}
              </span>
            </div>
            <div className="detail-item">
              <span className="detail-label">Time to Process:</span>
              <span className={`detail-value ${selectedSegmentData.ttp <= ttpThreshold ? 'pass' : 'fail'}`}>
                {selectedSegmentData.ttp.toFixed(2)}ms
                {selectedSegmentData.ttp <= ttpThreshold ? ' ✓' : ' ✗'}
              </span>
            </div>
            <div className="detail-item">
              <span className="detail-label">Contract Status:</span>
              <span className={`detail-value status-${selectedSegmentData.contractStatus}`}>
                {selectedSegmentData.contractStatus.toUpperCase()}
              </span>
            </div>
            <div className="detail-item">
              <span className="detail-label">Time Range:</span>
              <span className="detail-value">
                {selectedSegmentData.start.toFixed(2)}s - {selectedSegmentData.end.toFixed(2)}s
              </span>
            </div>
          </div>
        </div>
      )}

      <div className="contract-summary">
        <h4>Contract Summary</h4>
        <div className="summary-stats">
          <div className="stat-item">
            <span className="stat-label">DMR Threshold:</span>
            <span className="stat-value">≤ {(dmrThreshold * 100).toFixed(2)}%</span>
          </div>
          <div className="stat-item">
            <span className="stat-label">AJ Threshold:</span>
            <span className="stat-value">≤ {ajThreshold}ms</span>
          </div>
          <div className="stat-item">
            <span className="stat-label">TTP Threshold:</span>
            <span className="stat-value">≤ {ttpThreshold}ms</span>
          </div>
          <div className="stat-item">
            <span className="stat-label">TTB Threshold:</span>
            <span className="stat-value">≤ {ttbThreshold}ms</span>
          </div>
        </div>
        <div className="overall-status">
          <span className="status-label">Overall Contract:</span>
          <span className={`status-value ${profile.segments.every(s => s.contractStatus === 'pass') ? 'pass' : 'warning'}`}>
            {profile.segments.every(s => s.contractStatus === 'pass') ? 'PASS ✓' : 'WARNING ⚠'}
          </span>
        </div>
      </div>
    </div>
  );
};

