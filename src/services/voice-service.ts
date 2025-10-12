/**
 * Voice Service for Accessibility
 * Provides Text-to-Speech and Speech-to-Text functionality
 */

interface VoiceSettings {
  enabled: boolean;
  rate: number;
  pitch: number;
  volume: number;
  voice: string | null;
  language: string;
  autoRead: boolean;
}

class VoiceService {
  private synthesis: SpeechSynthesis;
  private recognition: any; // SpeechRecognition
  private settings: VoiceSettings;
  private isListening: boolean = false;
  private onCommandCallback: ((command: string) => void) | null = null;

  constructor() {
    this.synthesis = window.speechSynthesis;
    this.settings = {
      enabled: false,
      rate: 1.0,
      pitch: 1.0,
      volume: 1.0,
      voice: null,
      language: 'en-US',
      autoRead: false,
    };

    // Initialize Speech Recognition
    const SpeechRecognition = (window as any).SpeechRecognition || (window as any).webkitSpeechRecognition;
    if (SpeechRecognition) {
      this.recognition = new SpeechRecognition();
      this.recognition.continuous = true;
      this.recognition.interimResults = true;
      this.recognition.lang = this.settings.language;

      this.recognition.onresult = (event: any) => {
        const transcript = Array.from(event.results)
          .map((result: any) => result[0])
          .map((result: any) => result.transcript)
          .join('');

        if (event.results[0].isFinal && this.onCommandCallback) {
          this.onCommandCallback(transcript);
        }
      };

      this.recognition.onerror = (event: any) => {
        console.error('Speech recognition error:', event.error);
      };
    }
  }

  /**
   * Get available voices
   */
  getVoices(): SpeechSynthesisVoice[] {
    return this.synthesis.getVoices();
  }

  /**
   * Update voice settings
   */
  updateSettings(settings: Partial<VoiceSettings>) {
    this.settings = { ...this.settings, ...settings };
    if (this.recognition) {
      this.recognition.lang = this.settings.language;
    }
  }

  /**
   * Get current settings
   */
  getSettings(): VoiceSettings {
    return { ...this.settings };
  }

  /**
   * Speak text using Text-to-Speech
   */
  speak(text: string, priority: 'high' | 'normal' = 'normal'): void {
    if (!this.settings.enabled) return;

    // Cancel current speech if high priority
    if (priority === 'high') {
      this.synthesis.cancel();
    }

    const utterance = new SpeechSynthesisUtterance(text);
    utterance.rate = this.settings.rate;
    utterance.pitch = this.settings.pitch;
    utterance.volume = this.settings.volume;
    utterance.lang = this.settings.language;

    // Set voice if specified
    if (this.settings.voice) {
      const voices = this.getVoices();
      const voice = voices.find(v => v.name === this.settings.voice);
      if (voice) {
        utterance.voice = voice;
      }
    }

    this.synthesis.speak(utterance);
  }

  /**
   * Stop current speech
   */
  stop(): void {
    this.synthesis.cancel();
  }

  /**
   * Start listening for voice commands
   */
  startListening(onCommand: (command: string) => void): void {
    if (!this.recognition) {
      console.warn('Speech recognition not supported');
      return;
    }

    this.onCommandCallback = onCommand;
    this.isListening = true;
    this.recognition.start();
    this.speak('Voice commands activated', 'high');
  }

  /**
   * Stop listening for voice commands
   */
  stopListening(): void {
    if (!this.recognition || !this.isListening) return;

    this.isListening = false;
    this.recognition.stop();
    this.speak('Voice commands deactivated', 'high');
  }

  /**
   * Check if currently listening
   */
  getIsListening(): boolean {
    return this.isListening;
  }

  /**
   * Read code line
   */
  readCodeLine(lineNumber: number, code: string): void {
    if (!this.settings.enabled) return;
    this.speak(`Line ${lineNumber}: ${code}`);
  }

  /**
   * Read error message
   */
  readError(error: string): void {
    if (!this.settings.enabled) return;
    this.speak(`Error: ${error}`, 'high');
  }

  /**
   * Read notification
   */
  readNotification(message: string): void {
    if (!this.settings.enabled) return;
    this.speak(message);
  }

  /**
   * Read UI element
   */
  readElement(element: string, context?: string): void {
    if (!this.settings.enabled) return;
    const text = context ? `${element}, ${context}` : element;
    this.speak(text);
  }

  /**
   * Announce status change
   */
  announceStatus(status: string): void {
    if (!this.settings.enabled) return;
    this.speak(status, 'high');
  }

  /**
   * Enable voice features
   */
  enable(): void {
    this.settings.enabled = true;
    this.speak('Voice assistant enabled');
  }

  /**
   * Disable voice features
   */
  disable(): void {
    this.settings.enabled = false;
    this.stop();
    if (this.isListening) {
      this.stopListening();
    }
  }

  /**
   * Toggle voice features
   */
  toggle(): boolean {
    if (this.settings.enabled) {
      this.disable();
    } else {
      this.enable();
    }
    return this.settings.enabled;
  }
}

export const voiceService = new VoiceService();

