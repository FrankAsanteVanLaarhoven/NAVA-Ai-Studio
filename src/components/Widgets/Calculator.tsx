import React, { useState, useEffect } from 'react';
import './Calculator.css';

interface CalculatorProps {
  onClose?: () => void;
}

export const Calculator: React.FC<CalculatorProps> = ({ onClose }) => {
  const [display, setDisplay] = useState('0');
  const [previousValue, setPreviousValue] = useState<number | null>(null);
  const [operation, setOperation] = useState<string | null>(null);
  const [waitingForOperand, setWaitingForOperand] = useState(false);

  // Handle keyboard input
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key >= '0' && e.key <= '9') {
        inputNumber(e.key);
      } else if (e.key === '.') {
        inputDecimal();
      } else if (e.key === '+' || e.key === '-' || e.key === '*' || e.key === '/') {
        performOperation(e.key);
      } else if (e.key === 'Enter' || e.key === '=') {
        performOperation('=');
      } else if (e.key === 'Escape') {
        clear();
      } else if (e.key === 'Backspace') {
        setDisplay(display.length > 1 ? display.slice(0, -1) : '0');
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [display, previousValue, operation, waitingForOperand]);

  const inputNumber = (num: string) => {
    if (waitingForOperand) {
      setDisplay(num);
      setWaitingForOperand(false);
    } else {
      setDisplay(display === '0' ? num : display + num);
    }
  };

  const inputDecimal = () => {
    if (waitingForOperand) {
      setDisplay('0.');
      setWaitingForOperand(false);
    } else if (display.indexOf('.') === -1) {
      setDisplay(display + '.');
    }
  };

  const clear = () => {
    setDisplay('0');
    setPreviousValue(null);
    setOperation(null);
    setWaitingForOperand(false);
  };

  const performOperation = (nextOperation: string) => {
    const inputValue = parseFloat(display);

    if (previousValue === null) {
      setPreviousValue(inputValue);
    } else if (operation) {
      const currentValue = previousValue || 0;
      const newValue = calculate(currentValue, inputValue, operation);

      setDisplay(String(newValue));
      setPreviousValue(newValue);
    }

    setWaitingForOperand(true);
    setOperation(nextOperation === '=' ? null : nextOperation);
  };

  const calculate = (firstValue: number, secondValue: number, operation: string): number => {
    switch (operation) {
      case '+':
        return firstValue + secondValue;
      case '-':
        return firstValue - secondValue;
      case '*':
        return firstValue * secondValue;
      case '/':
        return firstValue / secondValue;
      default:
        return secondValue;
    }
  };

  // Button click handlers
  const handleNumberClick = (num: string) => {
    inputNumber(num);
  };

  const handleOperationClick = (op: string) => {
    performOperation(op);
  };

  const handleEqualsClick = () => {
    performOperation('=');
  };

  const handleClearClick = () => {
    clear();
  };

  const handleDecimalClick = () => {
    inputDecimal();
  };

  return (
    <div className="calculator-widget">
      <div className="calculator-header">
        <h3>Calculator</h3>
        {onClose && (
          <button className="calculator-close" onClick={onClose}>×</button>
        )}
      </div>
      
      <div className="calculator-display">
        <div className="display-value">{display}</div>
      </div>
      
      <div className="calculator-buttons">
        <button className="calculator-btn" onClick={handleClearClick}>C</button>
        <button className="calculator-btn" onClick={() => handleOperationClick('/')}>/</button>
        <button className="calculator-btn" onClick={() => handleOperationClick('*')}>×</button>
        <button className="calculator-btn" onClick={() => handleOperationClick('-')}>-</button>
        
        <button className="calculator-btn" onClick={() => handleNumberClick('7')}>7</button>
        <button className="calculator-btn" onClick={() => handleNumberClick('8')}>8</button>
        <button className="calculator-btn" onClick={() => handleNumberClick('9')}>9</button>
        <button className="calculator-btn" onClick={() => handleOperationClick('+')}>+</button>
        
        <button className="calculator-btn" onClick={() => handleNumberClick('4')}>4</button>
        <button className="calculator-btn" onClick={() => handleNumberClick('5')}>5</button>
        <button className="calculator-btn" onClick={() => handleNumberClick('6')}>6</button>
        <button className="calculator-btn" onClick={() => handleOperationClick('(')}>(</button>
        
        <button className="calculator-btn" onClick={() => handleNumberClick('1')}>1</button>
        <button className="calculator-btn" onClick={() => handleNumberClick('2')}>2</button>
        <button className="calculator-btn" onClick={() => handleNumberClick('3')}>3</button>
        <button className="calculator-btn" onClick={() => handleOperationClick(')')}>)</button>
        
        <button className="calculator-btn" onClick={() => handleNumberClick('0')}>0</button>
        <button className="calculator-btn" onClick={handleDecimalClick}>.</button>
        <button className="calculator-btn" onClick={handleEqualsClick}>=</button>
        <button className="calculator-btn" onClick={() => handleOperationClick('^')}>^</button>
      </div>
      
      <div className="calculator-footer">
        <small>Press ESC to clear, Backspace to delete</small>
      </div>
    </div>
  );
};