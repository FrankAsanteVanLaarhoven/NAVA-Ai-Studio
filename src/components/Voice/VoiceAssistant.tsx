import React, { useState, useEffect } from 'react';
import { Mic, MicOff, Volume2, VolumeX, Settings } from 'lucide-react';
import { voiceService } from '../../services/voice-service';
import './VoiceAssistant.css';

interface VoiceAssistantProps {
  onCommand?: (command: string) => void;
}

export const VoiceAssistant: React.FC<VoiceAssistantProps> = ({ onCommand }) => {
  const [isEnabled, setIsEnabled] = useState(false);
  const [isListening, setIsListening] = useState(false);
  const [transcript, setTranscript] = useState('');
  const [showSettings, setShowSettings] = useState(false);
  const [settings, setSettings] = useState(voiceService.getSettings());
  const [voices, setVoices] = useState<SpeechSynthesisVoice[]>([]);

  useEffect(() => {
    // Load available voices
    const loadVoices = () => {
      const availableVoices = voiceService.getVoices();
      setVoices(availableVoices);
    };

    loadVoices();
    if (speechSynthesis.onvoiceschanged !== undefined) {
      speechSynthesis.onvoiceschanged = loadVoices;
    }
  }, []);

  const handleToggleVoice = () => {
    const enabled = voiceService.toggle();
    setIsEnabled(enabled);
  };

  const handleToggleListening = () => {
    if (isListening) {
      voiceService.stopListening();
      setIsListening(false);
      setTranscript('');
    } else {
      voiceService.startListening((command) => {
        setTranscript(command);
        if (onCommand) {
          onCommand(command);
        }
        processVoiceCommand(command);
      });
      setIsListening(true);
    }
  };

  const processVoiceCommand = (command: string) => {
    const lowerCommand = command.toLowerCase();

    if (lowerCommand.includes('open settings')) {
      voiceService.speak('Opening settings');
      // Trigger settings open
    } else if (lowerCommand.includes('run code')) {
      voiceService.speak('Running code');
      // Trigger run
    } else if (lowerCommand.includes('save file')) {
      voiceService.speak('Saving file');
      // Trigger save
    } else if (lowerCommand.includes('open file')) {
      voiceService.speak('Opening file explorer');
      // Trigger file explorer
    } else if (lowerCommand.includes('compile')) {
      voiceService.speak('Compiling code');
      // Trigger compile
    } else if (lowerCommand.includes('help')) {
      voiceService.speak('Available commands: open settings, run code, save file, open file, compile, help');
    }
  };

  const handleSettingChange = (key: string, value: any) => {
    const newSettings = { ...settings, [key]: value };
    setSettings(newSettings);
    voiceService.updateSettings(newSettings);
  };

  return (
    <div className="voice-assistant">
      {/* Voice Controls */}
      <div className="voice-controls">
        <button
          className={`voice-btn ${isEnabled ? 'active' : ''}`}
          onClick={handleToggleVoice}
          title={isEnabled ? 'Disable Voice' : 'Enable Voice'}
        >
          {isEnabled ? <Volume2 size={18} /> : <VolumeX size={18} />}
        </button>

        <button
          className={`voice-btn mic-btn ${isListening ? 'listening' : ''}`}
          onClick={handleToggleListening}
          disabled={!isEnabled}
          title={isListening ? 'Stop Listening' : 'Start Voice Commands'}
        >
          {isListening ? <Mic size={18} /> : <MicOff size={18} />}
          {isListening && <span className="pulse-ring"></span>}
        </button>

        <button
          className="voice-btn"
          onClick={() => setShowSettings(!showSettings)}
          title="Voice Settings"
        >
          <Settings size={18} />
        </button>
      </div>

      {/* Listening Indicator */}
      {isListening && (
        <div className="listening-indicator">
          <div className="waveform">
            <span></span>
            <span></span>
            <span></span>
            <span></span>
            <span></span>
          </div>
          <div className="transcript">
            {transcript || 'Listening...'}
          </div>
        </div>
      )}

      {/* Settings Panel */}
      {showSettings && (
        <div className="voice-settings-panel">
          <h3>Voice Settings</h3>

          <div className="setting-group">
            <label>
              <input
                type="checkbox"
                checked={settings.autoRead}
                onChange={(e) => handleSettingChange('autoRead', e.target.checked)}
              />
              Auto-read notifications
            </label>
          </div>

          <div className="setting-group">
            <label>Voice</label>
            <select
              value={settings.voice || ''}
              onChange={(e) => handleSettingChange('voice', e.target.value)}
            >
              <option value="">Default</option>
              {voices.map((voice) => (
                <option key={voice.name} value={voice.name}>
                  {voice.name} ({voice.lang})
                </option>
              ))}
            </select>
          </div>

          <div className="setting-group">
            <label>Speed: {settings.rate.toFixed(1)}x</label>
            <input
              type="range"
              min="0.5"
              max="2.0"
              step="0.1"
              value={settings.rate}
              onChange={(e) => handleSettingChange('rate', parseFloat(e.target.value))}
            />
          </div>

          <div className="setting-group">
            <label>Pitch: {settings.pitch.toFixed(1)}</label>
            <input
              type="range"
              min="0.5"
              max="2.0"
              step="0.1"
              value={settings.pitch}
              onChange={(e) => handleSettingChange('pitch', parseFloat(e.target.value))}
            />
          </div>

          <div className="setting-group">
            <label>Volume: {Math.round(settings.volume * 100)}%</label>
            <input
              type="range"
              min="0"
              max="1"
              step="0.1"
              value={settings.volume}
              onChange={(e) => handleSettingChange('volume', parseFloat(e.target.value))}
            />
          </div>

          <div className="voice-commands-help">
            <h4>Voice Commands:</h4>
            <ul>
              <li>"Open settings"</li>
              <li>"Run code"</li>
              <li>"Save file"</li>
              <li>"Open file"</li>
              <li>"Compile"</li>
              <li>"Help"</li>
            </ul>
          </div>
        </div>
      )}
    </div>
  );
};

