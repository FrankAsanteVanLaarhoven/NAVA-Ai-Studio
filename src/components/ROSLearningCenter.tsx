import React, { useState, useEffect } from 'react';
import { allCourses, type Course, type Lesson } from '../services/ros-courses';
import { ROSTerminalService } from '../services/ros-terminal-service';
import './ROSLearningCenter.css';

interface ROSLearningCenterProps {
  onClose?: () => void;
}

export const ROSLearningCenter: React.FC<ROSLearningCenterProps> = ({ onClose }) => {
  const [selectedCourse, setSelectedCourse] = useState<Course | null>(null);
  const [selectedLesson, setSelectedLesson] = useState<Lesson | null>(null);
  const [terminalSessionId, setTerminalSessionId] = useState<string | null>(null);
  const [terminalOutput, setTerminalOutput] = useState<string>('');
  const [terminalInput, setTerminalInput] = useState<string>('');
  const [progress, setProgress] = useState<Record<string, string[]>>({});
  const [quizAnswers, setQuizAnswers] = useState<Record<number, number>>({});
  const [showQuizResults, setShowQuizResults] = useState(false);

  // Create terminal service instance
  const [rosTerminalService] = useState(() => new ROSTerminalService());

  // Initialize terminal session
  useEffect(() => {
    const sessionId = rosTerminalService.createSession();
    setTerminalSessionId(sessionId);
    updateTerminalOutput(sessionId);
  }, [rosTerminalService]);

  // Load progress from localStorage
  useEffect(() => {
    const savedProgress = localStorage.getItem('ros-learning-progress');
    if (savedProgress) {
      setProgress(JSON.parse(savedProgress));
    }
  }, []);

  // Save progress to localStorage
  const saveProgress = (courseId: string, lessonId: string) => {
    const newProgress = { ...progress };
    if (!newProgress[courseId]) {
      newProgress[courseId] = [];
    }
    if (!newProgress[courseId].includes(lessonId)) {
      newProgress[courseId].push(lessonId);
    }
    setProgress(newProgress);
    localStorage.setItem('ros-learning-progress', JSON.stringify(newProgress));
  };

  const updateTerminalOutput = (sessionId: string) => {
    const session = rosTerminalService.getSession(sessionId);
    if (session) {
      setTerminalOutput(session.output.join('\n'));
    }
  };

  // Calculate lesson progress for a course
  const getLessonProgress = (courseId: string): number => {
    const course = allCourses.find(c => c.id === courseId);
    if (!course) return 0;

    const completedLessons = progress[courseId] || [];
    const totalLessons = course.lessons.length;

    return totalLessons > 0 ? Math.round((completedLessons.length / totalLessons) * 100) : 0;
  };

  // Check if a lesson is completed
  const isLessonCompleted = (courseId: string, lessonId: string): boolean => {
    return progress[courseId]?.includes(lessonId) || false;
  };

  // Execute code example in terminal
  const executeCodeExample = async (code: string) => {
    if (!terminalSessionId) return;

    // Split code by lines and execute each command
    const commands = code.split('\n').filter(line => line.trim());
    for (const command of commands) {
      await rosTerminalService.executeCommand(terminalSessionId, command);
      updateTerminalOutput(terminalSessionId);
    }
  };

  // Handle quiz answer selection
  const handleQuizAnswer = (questionIndex: number, answerIndex: number) => {
    setQuizAnswers(prev => ({
      ...prev,
      [questionIndex]: answerIndex
    }));
  };

  // Submit quiz and calculate score
  const submitQuiz = () => {
    setShowQuizResults(true);
    if (selectedCourse && selectedLesson) {
      saveProgress(selectedCourse.id, selectedLesson.id);
    }
  };

  // Calculate quiz score
  const calculateQuizScore = (): number => {
    if (!selectedLesson?.quiz) return 0;

    let correct = 0;
    selectedLesson.quiz.forEach((question, index) => {
      if (quizAnswers[index] === question.correctAnswer) {
        correct++;
      }
    });

    return Math.round((correct / selectedLesson.quiz.length) * 100);
  };

  // Handle terminal command execution
  const handleTerminalCommand = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!terminalSessionId || !terminalInput.trim()) return;

    await rosTerminalService.executeCommand(terminalSessionId, terminalInput);
    updateTerminalOutput(terminalSessionId);
    setTerminalInput('');
  };

  return (
    <div className="ros-learning-center">
      <div className="ros-learning-header">
        <h1>🤖 ROS Learning Center</h1>
        <p>Master Robot Operating System with Interactive Courses</p>
        {onClose && (
          <button className="close-button" onClick={onClose}>
            ✕
          </button>
        )}
      </div>

      <div className="ros-learning-content">
        {/* Course List */}
        {!selectedCourse && (
          <div className="course-list">
            <h2>Available Courses</h2>
            <div className="courses-grid">
              {allCourses.map(course => (
                <div
                  key={course.id}
                  className="course-card"
                  style={{ borderLeft: `4px solid ${course.color}` }}
                  onClick={() => setSelectedCourse(course)}
                >
                  <div className="course-icon">{course.icon}</div>
                  <h3>{course.title}</h3>
                  <p className="course-description">{course.description}</p>
                  <div className="course-meta">
                    <span className={`level-badge level-${course.level}`}>
                      {course.level}
                    </span>
                    <span className="duration">⏱️ {course.duration}</span>
                  </div>
                  <div className="course-progress">
                    <div className="progress-bar">
                      <div
                        className="progress-fill"
                        style={{
                          width: `${getLessonProgress(course.id)}%`,
                          backgroundColor: course.color,
                        }}
                      />
                    </div>
                    <span className="progress-text">
                      {Math.round(getLessonProgress(course.id))}% Complete
                    </span>
                  </div>
                  <div className="course-outcomes">
                    <strong>You'll learn:</strong>
                    <ul>
                      {course.learningOutcomes.slice(0, 3).map((outcome, i) => (
                        <li key={i}>{outcome}</li>
                      ))}
                    </ul>
                  </div>
                </div>
              ))}
            </div>
          </div>
        )}

        {/* Lesson List */}
        {selectedCourse && !selectedLesson && (
          <div className="lesson-list">
            <button
              className="back-button"
              onClick={() => setSelectedCourse(null)}
            >
              ← Back to Courses
            </button>
            <h2>{selectedCourse.icon} {selectedCourse.title}</h2>
            <p className="course-description">{selectedCourse.description}</p>
            
            <div className="lessons-container">
              {selectedCourse.lessons.map((lesson, index) => (
                <div
                  key={lesson.id}
                  className={`lesson-card ${
                    isLessonCompleted(selectedCourse.id, lesson.id) ? 'completed' : ''
                  }`}
                  onClick={() => {
                    setSelectedLesson(lesson);
                    setQuizAnswers({});
                    setShowQuizResults(false);
                  }}
                >
                  <div className="lesson-number">{index + 1}</div>
                  <div className="lesson-info">
                    <h3>{lesson.title}</h3>
                    <p>{lesson.description}</p>
                    <span className="lesson-duration">⏱️ {lesson.duration}</span>
                  </div>
                  {isLessonCompleted(selectedCourse.id, lesson.id) && (
                    <div className="completion-badge">✓</div>
                  )}
                </div>
              ))}
            </div>
          </div>
        )}

        {/* Lesson Content */}
        {selectedCourse && selectedLesson && (
          <div className="lesson-viewer">
            <button
              className="back-button"
              onClick={() => setSelectedLesson(null)}
            >
              ← Back to Lessons
            </button>
            
            <div className="lesson-header">
              <h2>{selectedLesson.title}</h2>
              <span className="lesson-duration">⏱️ {selectedLesson.duration}</span>
            </div>

            <div className="lesson-content-area">
              {/* Lesson Content */}
              <div className="lesson-text">
                <div
                  dangerouslySetInnerHTML={{
                    __html: selectedLesson.content
                      .replace(/\n/g, '<br/>')
                      .replace(/```(.*?)```/gs, '<pre><code>$1</code></pre>')
                      .replace(/## (.*?)(<br\/>|$)/g, '<h2>$1</h2>')
                      .replace(/### (.*?)(<br\/>|$)/g, '<h3>$1</h3>')
                      .replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>')
                      .replace(/`(.*?)`/g, '<code>$1</code>'),
                  }}
                />
              </div>

              {/* Code Examples */}
              {selectedLesson.codeExamples.length > 0 && (
                <div className="code-examples">
                  <h3>💻 Code Examples</h3>
                  {selectedLesson.codeExamples.map((example, index) => (
                    <div key={index} className="code-example">
                      <div className="code-example-header">
                        <span className="language-badge">{example.language}</span>
                        <span className="example-explanation">{example.explanation}</span>
                        {example.executable && example.command && (
                          <button
                            className="run-button"
                            onClick={() => executeCodeExample(example.command!)}
                          >
                            ▶ Run
                          </button>
                        )}
                      </div>
                      <pre className="code-block">
                        <code>{example.code}</code>
                      </pre>
                    </div>
                  ))}
                </div>
              )}

              {/* Practice Exercises */}
              {selectedLesson.practiceExercises && selectedLesson.practiceExercises.length > 0 && (
                <div className="practice-exercises">
                  <h3>🎯 Practice Exercises</h3>
                  <ul>
                    {selectedLesson.practiceExercises.map((exercise, index) => (
                      <li key={index}>{exercise}</li>
                    ))}
                  </ul>
                </div>
              )}

              {/* Quiz */}
              {selectedLesson.quiz && selectedLesson.quiz.length > 0 && (
                <div className="quiz-section">
                  <h3>📝 Knowledge Check</h3>
                  {selectedLesson.quiz.map((question, qIndex) => (
                    <div key={qIndex} className="quiz-question">
                      <p className="question-text">
                        <strong>Q{qIndex + 1}:</strong> {question.question}
                      </p>
                      <div className="quiz-options">
                        {question.options.map((option, oIndex) => (
                          <label
                            key={oIndex}
                            className={`quiz-option ${
                              showQuizResults
                                ? oIndex === question.correctAnswer
                                  ? 'correct'
                                  : quizAnswers[qIndex] === oIndex
                                  ? 'incorrect'
                                  : ''
                                : quizAnswers[qIndex] === oIndex
                                ? 'selected'
                                : ''
                            }`}
                          >
                            <input
                              type="radio"
                              name={`question-${qIndex}`}
                              checked={quizAnswers[qIndex] === oIndex}
                              onChange={() => handleQuizAnswer(qIndex, oIndex)}
                              disabled={showQuizResults}
                            />
                            <span>{option}</span>
                          </label>
                        ))}
                      </div>
                      {showQuizResults && (
                        <div className="quiz-explanation">
                          <strong>Explanation:</strong> {question.explanation}
                        </div>
                      )}
                    </div>
                  ))}
                  
                  {!showQuizResults && (
                    <button
                      className="submit-quiz-button"
                      onClick={submitQuiz}
                      disabled={Object.keys(quizAnswers).length !== selectedLesson.quiz.length}
                    >
                      Submit Quiz
                    </button>
                  )}
                  
                  {showQuizResults && (
                    <div className="quiz-results">
                      <h4>Quiz Results</h4>
                      <p className="score">
                        Score: {calculateQuizScore().toFixed(0)}%
                      </p>
                      <p>
                        {calculateQuizScore() >= 80
                          ? '🎉 Great job! You can move on to the next lesson.'
                          : '📚 Review the material and try again to improve your score.'}
                      </p>
                    </div>
                  )}
                </div>
              )}
            </div>
          </div>
        )}
      </div>

      {/* Terminal */}
      <div className="ros-terminal">
        <div className="terminal-header">
          <span>🖥️ ROS Terminal</span>
          <button
            className="clear-terminal"
            onClick={() => {
              if (terminalSessionId) {
                rosTerminalService.clearHistory(terminalSessionId);
                updateTerminalOutput(terminalSessionId);
              }
            }}
          >
            Clear
          </button>
        </div>
        <div className="terminal-output">
          <pre>{terminalOutput}</pre>
        </div>
        <form className="terminal-input-form" onSubmit={handleTerminalCommand}>
          <span className="terminal-prompt">$</span>
          <input
            type="text"
            className="terminal-input"
            value={terminalInput}
            onChange={(e) => setTerminalInput(e.target.value)}
            placeholder="Type a ROS command..."
            autoComplete="off"
          />
        </form>
      </div>
    </div>
  );
};
