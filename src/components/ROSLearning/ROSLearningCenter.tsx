import React, { useState, useEffect } from 'react';
import { rosEducationMCP, ROSCourse, ROSModule, ROSUnit } from '../../services/ros-education-mcp';
import { ROSTerminal } from './ROSTerminal';
import './ROSLearningCenter.css';

export const ROSLearningCenter: React.FC = () => {
  const [courses, setCourses] = useState<ROSCourse[]>([]);
  const [selectedCourse, setSelectedCourse] = useState<ROSCourse | null>(null);
  const [selectedModule, setSelectedModule] = useState<ROSModule | null>(null);
  const [selectedUnit, setSelectedUnit] = useState<ROSUnit | null>(null);
  const [userId] = useState('demo-user'); // TODO: Get from auth system
  const [showTerminal, setShowTerminal] = useState(false);
  const [terminalCommand, setTerminalCommand] = useState<string | undefined>(undefined);
  
  useEffect(() => {
    // Load free courses
    const freeCourses = rosEducationMCP.getFreeCourses();
    setCourses(freeCourses);
  }, []);
  
  const enrollInCourse = (courseId: string) => {
    const progress = rosEducationMCP.enrollCourse(userId, courseId);
    console.log('Enrolled in course:', courseId, progress);
    alert(`✅ Enrolled in course! Your progress will be saved.`);
  };
  
  const markUnitComplete = (unitId: string) => {
    if (selectedCourse) {
      rosEducationMCP.updateProgress(userId, selectedCourse.id, unitId, true);
      alert(`✅ Unit completed! Keep going!`);
    }
  };
  
  const runROSCommand = (command: string) => {
    console.log('Running ROS command:', command);
    setTerminalCommand(command);
    setShowTerminal(true);
  };
  
  const handleCloseTerminal = () => {
    setShowTerminal(false);
    setTerminalCommand(undefined);
  };
  
  return (
    <div className="ros-learning-center">
      <div className="ros-header" style={{ flexShrink: 0 }}>
        <div className="ros-header-content">
          <div className="ros-header-text">
            <h1>🤖 ROS Learning Center</h1>
            <p>Free, comprehensive Robot Operating System courses integrated with NAVΛ</p>
          </div>
          <button 
            className="open-terminal-btn"
            onClick={() => setShowTerminal(true)}
            title="Open ROS2 Terminal"
          >
            💻 Open Terminal
          </button>
        </div>
        <div className="ros-stats">
          <div className="stat">
            <span className="stat-number">{courses.length}</span>
            <span className="stat-label">Free Courses</span>
          </div>
          <div className="stat">
            <span className="stat-number">100%</span>
            <span className="stat-label">Free Forever</span>
          </div>
          <div className="stat">
            <span className="stat-number">✅</span>
            <span className="stat-label">Certification</span>
          </div>
        </div>
      </div>
      
      {!selectedCourse && (
        <div className="courses-grid">
          {courses.map(course => (
            <div key={course.id} className="course-card">
              <div className="course-header">
                <span className="course-badge">{course.difficulty}</span>
                <span className="course-badge free">FREE</span>
                {course.certification && <span className="course-badge cert">🎓 Cert</span>}
              </div>
              
              <h2>{course.title}</h2>
              <p className="course-description">{course.description}</p>
              
              <div className="course-meta">
                <span>⏱️ {course.duration}</span>
                <span>📦 {course.rosVersion}</span>
                <span>📚 {course.modules.length} modules</span>
              </div>
              
              <div className="course-outcomes">
                <h4>You'll learn:</h4>
                <ul>
                  {course.learningOutcomes.slice(0, 3).map((outcome, i) => (
                    <li key={i}>{outcome}</li>
                  ))}
                </ul>
              </div>
              
              <div className="course-actions">
                <button 
                  className="btn-primary"
                  onClick={() => {
                    enrollInCourse(course.id);
                    setSelectedCourse(course);
                  }}
                >
                  Start Learning →
                </button>
                <button 
                  className="btn-secondary"
                  onClick={() => setSelectedCourse(course)}
                >
                  View Syllabus
                </button>
              </div>
            </div>
          ))}
        </div>
      )}
      
      {selectedCourse && !selectedModule && (
        <div className="course-detail" style={{ flex: 1, overflowY: 'auto', overflowX: 'hidden' }}>
          <button className="back-btn" onClick={() => setSelectedCourse(null)}>
            ← Back to Courses
          </button>
          
          <div className="course-detail-header">
            <h1>{selectedCourse.title}</h1>
            <p>{selectedCourse.description}</p>
            
            <div className="course-info-grid">
              <div className="info-item">
                <strong>Difficulty:</strong> {selectedCourse.difficulty}
              </div>
              <div className="info-item">
                <strong>Duration:</strong> {selectedCourse.duration}
              </div>
              <div className="info-item">
                <strong>ROS Version:</strong> {selectedCourse.rosVersion}
              </div>
              <div className="info-item">
                <strong>Prerequisites:</strong> {
                  selectedCourse.prerequisites.length > 0 
                    ? selectedCourse.prerequisites.join(', ')
                    : 'None'
                }
              </div>
            </div>
          </div>
          
          <div className="modules-list">
            <h2>Course Modules</h2>
            {selectedCourse.modules.map((module, index) => (
              <div key={module.id} className="module-card">
                <div className="module-header">
                  <h3>
                    <span className="module-number">Module {index + 1}</span>
                    {module.title}
                  </h3>
                  <span className="module-time">⏱️ {module.estimatedTime}</span>
                </div>
                
                <p>{module.description}</p>
                
                <div className="module-units">
                  <h4>{module.units.length} Units:</h4>
                  <ul>
                    {module.units.map(unit => (
                      <li key={unit.id}>
                        <span className={`unit-type ${unit.type}`}>{unit.type}</span>
                        {unit.title}
                        <span className="unit-time">({unit.estimatedTime})</span>
                      </li>
                    ))}
                  </ul>
                </div>
                
                {module.rosjects.length > 0 && (
                  <div className="module-rosjects">
                    <h4>🚀 ROSjects (Hands-on Projects):</h4>
                    <ul>
                      {module.rosjects.map(rosject => (
                        <li key={rosject.id}>
                          <strong>{rosject.title}</strong>
                          <p>{rosject.description}</p>
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
                
                <button 
                  className="btn-primary"
                  onClick={() => setSelectedModule(module)}
                >
                  Start Module {index + 1} →
                </button>
              </div>
            ))}
          </div>
        </div>
      )}
      
      {selectedModule && (
        <div className="module-detail">
          <button className="back-btn" onClick={() => setSelectedModule(null)}>
            ← Back to Course
          </button>
          
          <div className="module-detail-header">
            <h1>{selectedModule.title}</h1>
            <p>{selectedModule.description}</p>
          </div>
          
          <div className="units-grid">
            {selectedModule.units.map((unit, index) => (
              <div key={unit.id} className="unit-card">
                <div className="unit-header">
                  <span className="unit-number">Unit {index + 1}</span>
                  <span className={`unit-type-badge ${unit.type}`}>
                    {unit.type}
                  </span>
                </div>
                
                <h3>{unit.title}</h3>
                <span className="unit-time">⏱️ {unit.estimatedTime}</span>
                
                <button 
                  className="btn-primary"
                  onClick={() => setSelectedUnit(unit)}
                >
                  Start Unit →
                </button>
              </div>
            ))}
          </div>
        </div>
      )}
      
      {selectedUnit && (
        <div className="unit-viewer" style={{ flex: 1, overflowY: 'auto', overflowX: 'hidden' }}>
          <button 
            className="back-btn" 
            onClick={() => setSelectedUnit(null)}
          >
            ← Back to Module
          </button>
          
          <div className="unit-content">
            <div className="unit-header-section">
              <h1>{selectedUnit.title}</h1>
              <span className={`unit-type-badge ${selectedUnit.type}`}>
                {selectedUnit.type}
              </span>
              <span className="unit-time">⏱️ {selectedUnit.estimatedTime}</span>
            </div>
            
            <div 
              className="unit-markdown-content"
              dangerouslySetInnerHTML={{ __html: selectedUnit.content }}
            />
            
            {selectedUnit.codeExamples.length > 0 && (
              <div className="code-examples">
                <h2>💻 Code Examples</h2>
                {selectedUnit.codeExamples.map((example, i) => (
                  <div key={i} className="code-example">
                    <div className="code-example-header">
                      <h3>{example.title}</h3>
                      <span className="code-language">{example.language}</span>
                    </div>
                    
                    <p className="code-description">{example.description}</p>
                    
                    <pre className="code-block">
                      <code className={`language-${example.language}`}>
                        {example.code}
                      </code>
                    </pre>
                    
                    {example.language === 'bash' && (
                      <button 
                        className="run-btn"
                        onClick={() => runROSCommand(example.code)}
                      >
                        ▶️ Run Command
                      </button>
                    )}
                    
                    <div className="code-explanation">
                      <strong>Explanation:</strong> {example.explanation}
                    </div>
                  </div>
                ))}
              </div>
            )}
            
            {selectedUnit.launchFiles.length > 0 && (
              <div className="launch-files">
                <h2>🚀 Launch Files</h2>
                {selectedUnit.launchFiles.map((launchFile, i) => (
                  <div key={i} className="launch-file">
                    <div className="launch-file-header">
                      <h3>{launchFile.name}</h3>
                      <span className="package-badge">{launchFile.packageName}</span>
                    </div>
                    
                    <p>{launchFile.description}</p>
                    
                    <pre className="code-block">
                      <code className="language-python">
                        {launchFile.content}
                      </code>
                    </pre>
                    
                    <button 
                      className="run-btn"
                      onClick={() => runROSCommand(
                        `ros2 launch ${launchFile.packageName} ${launchFile.name}`
                      )}
                    >
                      🚀 Launch
                    </button>
                  </div>
                ))}
              </div>
            )}
            
            <div className="unit-actions">
              <button 
                className="btn-success"
                onClick={() => markUnitComplete(selectedUnit.id)}
              >
                ✅ Mark as Complete
              </button>
              
              <button 
                className="btn-secondary"
                onClick={() => {
                  // Go to next unit
                  setSelectedUnit(null);
                }}
              >
                Next Unit →
              </button>
            </div>
          </div>
        </div>
      )}
      
      {/* ROS Terminal */}
      <ROSTerminal 
        isOpen={showTerminal}
        onClose={handleCloseTerminal}
        initialCommand={terminalCommand}
      />
    </div>
  );
};

