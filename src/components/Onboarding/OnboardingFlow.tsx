import React, { useState } from 'react';
import './OnboardingFlow.css';

interface OnboardingStep {
  id: string;
  title: string;
  description: string;
  content: React.ReactNode;
  action?: {
    label: string;
    handler: () => void;
  };
}

export const OnboardingFlow: React.FC = () => {
  const [currentStep, setCurrentStep] = useState(0);
  const [isVisible, setIsVisible] = useState(!localStorage.getItem('onboarding_completed'));

  const steps: OnboardingStep[] = [
    {
      id: 'welcome',
      title: 'Welcome to NAVΛ Studio',
      description: 'The world\'s first IDE designed specifically for navigation calculus',
      content: (
        <div className="welcome-content">
          <div className="lambda-logo">⋋</div>
          <h2>Van Laarhoven Navigation Calculus</h2>
          <p>
            NAVΛ Studio combines mathematical rigor with production-grade tooling,
            enabling you to write, visualize, and deploy navigation algorithms
            with unprecedented ease.
          </p>
          <div className="feature-grid">
            <div className="feature-card">
              <span className="feature-icon">🎨</span>
              <h3>Visual Development</h3>
              <p>See your navigation fields in real-time 3D</p>
            </div>
            <div className="feature-card">
              <span className="feature-icon">⚡</span>
              <h3>Multi-Target Build</h3>
              <p>Compile to C++, WASM, Python, GLSL</p>
            </div>
            <div className="feature-card">
              <span className="feature-icon">🤖</span>
              <h3>AI Assistant</h3>
              <p>Get help from specialized AI models</p>
            </div>
            <div className="feature-card">
              <span className="feature-icon">☁️</span>
              <h3>Cloud Deploy</h3>
              <p>One-click deployment to production</p>
            </div>
          </div>
        </div>
      ),
    },
    {
      id: 'interface',
      title: 'IDE Interface Tour',
      description: 'Learn the layout and key features',
      content: (
        <div className="interface-tour">
          <div className="tour-section">
            <h3>📝 Code Editor</h3>
            <p>
              Monaco-powered editor with full VNC syntax support, IntelliSense,
              and real-time error checking. Write navigation code with
              mathematical precision.
            </p>
          </div>
          <div className="tour-section">
            <h3>🎬 Live Preview</h3>
            <p>
              See your navigation fields visualized in real-time. Rotate, zoom,
              and interact with 3D manifolds as you code.
            </p>
          </div>
          <div className="tour-section">
            <h3>🔍 Navigation Gutter</h3>
            <p>
              Inline visualization of navigation properties. Hover over symbols
              to see their geometric interpretation.
            </p>
          </div>
          <div className="tour-section">
            <h3>🐛 Debugger</h3>
            <p>
              Step through navigation algorithms, inspect manifold states,
              and profile performance with specialized tools.
            </p>
          </div>
        </div>
      ),
    },
    {
      id: 'first-program',
      title: 'Your First Navigation Program',
      description: 'Let\'s create a simple navigation field',
      content: (
        <div className="first-program">
          <h3>Example: 2D Navigation Field</h3>
          <pre className="code-sample">
{`// Define a 2D navigation field
⋋ field: ℝ² → TM = {
    // Position in 2D space
    position: vector2(x, y),
    
    // Goal position
    goal: vector2(10, 10),
    
    // Energy function (distance to goal)
    energy: |position - goal|²
}

// Find optimal path from origin to goal
⋋ path = navigate(
    field: field,
    start: vector2(0, 0),
    goal: vector2(10, 10),
    method: "gradient_descent"
)

// Visualize the result
@visualize(path, field)`}
          </pre>
          <button className="try-it-btn" onClick={() => createSampleProject()}>
            🚀 Try This Example
          </button>
        </div>
      ),
      action: {
        label: 'Create Sample Project',
        handler: createSampleProject,
      },
    },
    {
      id: 'templates',
      title: 'Project Templates',
      description: 'Start with pre-built examples',
      content: (
        <div className="templates-grid">
          <div className="template-card">
            <span className="template-icon">🤖</span>
            <h3>Robot Navigation</h3>
            <p>Path planning for autonomous robots with obstacle avoidance</p>
            <button className="template-btn">Use Template</button>
          </div>
          <div className="template-card">
            <span className="template-icon">🧠</span>
            <h3>ML-Guided Optimization</h3>
            <p>Neural network-assisted navigation field optimization</p>
            <button className="template-btn">Use Template</button>
          </div>
          <div className="template-card">
            <span className="template-icon">⚛️</span>
            <h3>Quantum Navigation</h3>
            <p>Quantum state evolution on Hilbert space manifolds</p>
            <button className="template-btn">Use Template</button>
          </div>
          <div className="template-card">
            <span className="template-icon">🌌</span>
            <h3>Physics Simulation</h3>
            <p>Particle dynamics on curved spacetime manifolds</p>
            <button className="template-btn">Use Template</button>
          </div>
          <div className="template-card">
            <span className="template-icon">📊</span>
            <h3>Data Optimization</h3>
            <p>Gradient-based optimization on data manifolds</p>
            <button className="template-btn">Use Template</button>
          </div>
          <div className="template-card">
            <span className="template-icon">🎮</span>
            <h3>Game AI</h3>
            <p>Intelligent agent navigation in game environments</p>
            <button className="template-btn">Use Template</button>
          </div>
        </div>
      ),
    },
    {
      id: 'ai-assistant',
      title: 'AI-Powered Development',
      description: 'Meet your AI coding assistant',
      content: (
        <div className="ai-assistant-intro">
          <div className="ai-demo">
            <div className="ai-avatar">🤖</div>
            <div className="ai-message">
              <p>
                <strong>Hi! I'm your NAVΛ AI Assistant.</strong>
              </p>
              <p>
                I can help you with:
              </p>
              <ul>
                <li>Writing and debugging VNC code</li>
                <li>Explaining navigation calculus concepts</li>
                <li>Optimizing your algorithms</li>
                <li>Suggesting best practices</li>
                <li>Generating visualizations</li>
              </ul>
              <p>
                Just click the 🤖 icon in the sidebar or press <kbd>Ctrl+I</kbd>
                to chat with me anytime!
              </p>
            </div>
          </div>
          <div className="ai-models">
            <h3>Choose Your AI Model</h3>
            <p>Select from powerful models like Claude, GPT-4, or Gemini</p>
            <button className="setup-ai-btn" onClick={() => window.open('/app.html#ai-panel')}>
              Set Up AI Assistant
            </button>
          </div>
        </div>
      ),
    },
    {
      id: 'resources',
      title: 'Learning Resources',
      description: 'Continue your NAVΛ journey',
      content: (
        <div className="resources-section">
          <h3>📚 Documentation</h3>
          <ul className="resource-list">
            <li>
              <a href="/docs/vnc-language-reference.md">VNC Language Reference</a>
              <span>Complete syntax and semantics guide</span>
            </li>
            <li>
              <a href="/docs/architecture.md">Architecture Overview</a>
              <span>Understand how NAVΛ Studio works</span>
            </li>
            <li>
              <a href="/docs/plugin-development.md">Plugin Development</a>
              <span>Extend NAVΛ with custom functionality</span>
            </li>
          </ul>

          <h3>🎥 Video Tutorials</h3>
          <ul className="resource-list">
            <li>
              <a href="#">Getting Started (5 min)</a>
              <span>Quick introduction to NAVΛ Studio</span>
            </li>
            <li>
              <a href="#">Navigation Fields (15 min)</a>
              <span>Deep dive into field theory</span>
            </li>
            <li>
              <a href="#">Advanced Optimization (30 min)</a>
              <span>Professional optimization techniques</span>
            </li>
          </ul>

          <h3>🌍 Community</h3>
          <ul className="resource-list">
            <li>
              <a href="#">NAVΛ Forum</a>
              <span>Ask questions and share projects</span>
            </li>
            <li>
              <a href="#">Discord Server</a>
              <span>Real-time chat with the community</span>
            </li>
            <li>
              <a href="#">GitHub Repository</a>
              <span>Contribute to the platform</span>
            </li>
          </ul>
        </div>
      ),
    },
    {
      id: 'complete',
      title: 'You\'re All Set!',
      description: 'Ready to build amazing navigation systems',
      content: (
        <div className="completion-content">
          <div className="success-icon">✨</div>
          <h2>Welcome to the NAVΛ Community!</h2>
          <p className="completion-message">
            You're now ready to explore the full power of navigation calculus.
            Remember, you can always access this tutorial from the Help menu.
          </p>
          <div className="quick-actions">
            <button className="action-btn primary" onClick={createNewProject}>
              📄 Create New Project
            </button>
            <button className="action-btn" onClick={() => window.open('/docs')}>
              📖 View Documentation
            </button>
            <button className="action-btn" onClick={openTemplates}>
              📦 Browse Templates
            </button>
          </div>
          <div className="tips-box">
            <h3>💡 Pro Tips</h3>
            <ul>
              <li><kbd>Ctrl+Space</kbd> - Trigger IntelliSense</li>
              <li><kbd>Ctrl+I</kbd> - Open AI Assistant</li>
              <li><kbd>Ctrl+Shift+V</kbd> - Toggle Live Preview</li>
              <li><kbd>F5</kbd> - Run/Debug Current File</li>
            </ul>
          </div>
        </div>
      ),
    },
  ];

  const completeOnboarding = () => {
    localStorage.setItem('onboarding_completed', 'true');
    setIsVisible(false);
  };

  const nextStep = () => {
    if (currentStep < steps.length - 1) {
      setCurrentStep(currentStep + 1);
    } else {
      completeOnboarding();
    }
  };

  const previousStep = () => {
    if (currentStep > 0) {
      setCurrentStep(currentStep - 1);
    }
  };

  const skipOnboarding = () => {
    completeOnboarding();
  };

  if (!isVisible) {
    return null;
  }

  const currentStepData = steps[currentStep];
  const progress = ((currentStep + 1) / steps.length) * 100;

  return (
    <div className="onboarding-overlay">
      <div className="onboarding-modal">
        <div className="onboarding-header">
          <div className="step-indicator">
            Step {currentStep + 1} of {steps.length}
          </div>
          <button className="skip-btn" onClick={skipOnboarding}>
            Skip Tutorial
          </button>
        </div>

        <div className="progress-bar">
          <div className="progress-fill" style={{ width: `${progress}%` }} />
        </div>

        <div className="onboarding-content">
          <h1>{currentStepData.title}</h1>
          <p className="step-description">{currentStepData.description}</p>
          <div className="step-content">{currentStepData.content}</div>
        </div>

        <div className="onboarding-footer">
          <button
            className="nav-btn"
            onClick={previousStep}
            disabled={currentStep === 0}
          >
            ← Previous
          </button>

          <div className="step-dots">
            {steps.map((_, index) => (
              <span
                key={index}
                className={`dot ${index === currentStep ? 'active' : ''} ${
                  index < currentStep ? 'completed' : ''
                }`}
                onClick={() => setCurrentStep(index)}
              />
            ))}
          </div>

          <button className="nav-btn primary" onClick={nextStep}>
            {currentStep === steps.length - 1 ? 'Get Started' : 'Next →'}
          </button>
        </div>
      </div>
    </div>
  );
};

// Helper functions
function createSampleProject() {
  // TODO: Implement sample project creation
  console.log('Creating sample navigation project...');
}

function createNewProject() {
  // TODO: Implement new project creation
  console.log('Creating new project...');
}

function openTemplates() {
  // TODO: Implement template browser
  console.log('Opening template browser...');
}

