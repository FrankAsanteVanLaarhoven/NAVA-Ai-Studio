import React, { useState } from 'react';
import './Calculator.css';

interface CalculatorProps {
  onClose?: () => void;
}

export const Calculator: React.FC<CalculatorProps> = ({ onClose }) => {
  const [display, setDisplay] = useState('0');
  const [previousValue, setPreviousValue] = useState<number | null>(null);
  const [operation, setOperation] = useState<string | null>(null);
  const [waitingForOperand, setWaitingForOperand] = useState(false);

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
    setOperation(nextOperation);
  };

  const calculate = (firstValue: number, secondValue: number, operation: string): number => {
    switch (operation) {
      case '+':
        return firstValue + secondValue;
      case '-':
        return firstValue - secondValue;
      case '×':
        return firstValue * secondValue;
      case '÷':
        return firstValue / secondValue;
      case '=':
        return secondValue;
      default:
        return secondValue;
    }
  };

  const handleKeyPress = (event: React.KeyboardEvent) => {
    const { key } = event;
    
    if (key >= '0' && key <= '9') {
      inputNumber(key);
    } else if (key === '.') {
      inputDecimal();
    } else if (key === '+' || key === '-') {
      performOperation(key);
    } else if (key === '*') {
      performOperation('×');
    } else if (key === '/') {
      event.preventDefault();
      performOperation('÷');
    } else if (key === 'Enter' || key === '=') {
      performOperation('=');
    } else if (key === 'Escape' || key === 'c' || key === 'C') {
      clear();
    }
  };

  return (
    <div className="calculator-widget" onKeyDown={handleKeyPress} tabIndex={0}>
      <div className="calculator-header">
        <h3>Calculator</h3>
        {onClose && (
          <button className="calculator-close" onClick={onClose}>
            ×
          </button>
        )}
      </div>
      
      <div className="calculator-display">
        {display}
      </div>
      
      <div className="calculator-buttons">
        <button className="calculator-btn clear" onClick={clear}>
          C
        </button>
        <button className="calculator-btn operation" onClick={() => performOperation('÷')}>
          ÷
        </button>
        <button className="calculator-btn operation" onClick={() => performOperation('×')}>
          ×
        </button>
        <button className="calculator-btn operation" onClick={() => performOperation('-')}>
          -
        </button>
        
        <button className="calculator-btn number" onClick={() => inputNumber('7')}>
          7
        </button>
        <button className="calculator-btn number" onClick={() => inputNumber('8')}>
          8
        </button>
        <button className="calculator-btn number" onClick={() => inputNumber('9')}>
          9
        </button>
        <button className="calculator-btn operation plus" onClick={() => performOperation('+')}>
          +
        </button>
        
        <button className="calculator-btn number" onClick={() => inputNumber('4')}>
          4
        </button>
        <button className="calculator-btn number" onClick={() => inputNumber('5')}>
          5
        </button>
        <button className="calculator-btn number" onClick={() => inputNumber('6')}>
          6
        </button>
        
        <button className="calculator-btn number" onClick={() => inputNumber('1')}>
          1
        </button>
        <button className="calculator-btn number" onClick={() => inputNumber('2')}>
          2
        </button>
        <button className="calculator-btn number" onClick={() => inputNumber('3')}>
          3
        </button>
        <button className="calculator-btn equals" onClick={() => performOperation('=')}>
          =
        </button>
        
        <button className="calculator-btn number zero" onClick={() => inputNumber('0')}>
          0
        </button>
        <button className="calculator-btn decimal" onClick={inputDecimal}>
          .
        </button>
      </div>
      
      <div className="calculator-footer">
        <small>Use keyboard for input • ESC to clear</small>
      </div>
    </div>
  );
};