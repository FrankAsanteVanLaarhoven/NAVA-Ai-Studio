import React, { useEffect, useState } from 'react';

interface WebAssemblyRunnerProps {
  wasmCode: Uint8Array | null;
  onResult: (result: any) => void;
}

export const WebAssemblyRunner: React.FC<WebAssemblyRunnerProps> = ({
  wasmCode,
  onResult,
}) => {
  const [status, setStatus] = useState<'idle' | 'loading' | 'running' | 'error'>('idle');
  const [memory, setMemory] = useState<number>(0);

  useEffect(() => {
    if (wasmCode) {
      runWasm();
    }
  }, [wasmCode]);

  const runWasm = async () => {
    if (!wasmCode) return;

    setStatus('loading');

    try {
      const module = await WebAssembly.instantiate(wasmCode, {
        env: {
          log: (value: number) => console.log('WASM log:', value),
        },
      });

      setStatus('running');

      // Execute main function if it exists
      if (typeof module.instance.exports.main === 'function') {
        const result = (module.instance.exports.main as Function)();
        onResult(result);
      }

      // Update memory usage
      if (module.instance.exports.memory) {
        const mem = module.instance.exports.memory as WebAssembly.Memory;
        setMemory(mem.buffer.byteLength);
      }

      setStatus('idle');
    } catch (error) {
      console.error('WASM execution error:', error);
      setStatus('error');
    }
  };

  return (
    <div className="wasm-runner">
      <div className="wasm-status">
        Status: <span className={`status-${status}`}>{status}</span>
      </div>
      <div className="wasm-memory">
        Memory: {(memory / 1024).toFixed(2)} KB
      </div>
    </div>
  );
};

