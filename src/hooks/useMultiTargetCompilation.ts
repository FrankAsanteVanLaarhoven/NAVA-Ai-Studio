import { useState, useCallback } from 'react';
import { CompilationTarget, CompilationResult } from '../types';
import { compilerService } from '../services/compiler-service';

export function useMultiTargetCompilation() {
  const [compilationResults, setCompilationResults] = useState<CompilationResult[]>([]);
  const [isCompiling, setIsCompiling] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const compileToTarget = useCallback(async (code: string, target: CompilationTarget) => {
    setIsCompiling(true);
    setError(null);

    try {
      const result = await compilerService.compileToTarget(code, target);
      setCompilationResults((prev) => [...prev.filter((r) => r.target !== target), result]);
      return result;
    } catch (err) {
      setError(String(err));
      throw err;
    } finally {
      setIsCompiling(false);
    }
  }, []);

  const compileToAllTargets = useCallback(async (code: string) => {
    setIsCompiling(true);
    setError(null);

    try {
      const results = await compilerService.compileToAllTargets(code);
      setCompilationResults(results);
      return results;
    } catch (err) {
      setError(String(err));
      throw err;
    } finally {
      setIsCompiling(false);
    }
  }, []);

  const clearResults = useCallback(() => {
    setCompilationResults([]);
    setError(null);
  }, []);

  return {
    compilationResults,
    isCompiling,
    error,
    compileToTarget,
    compileToAllTargets,
    clearResults,
  };
}

