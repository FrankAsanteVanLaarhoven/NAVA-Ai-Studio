export interface Task {
  id: string;
  label: string;
  type: 'shell' | 'process' | 'npm' | 'python' | 'custom';
  command: string;
  args?: string[];
  options?: {
    cwd?: string;
    env?: Record<string, string>;
    shell?: string;
  };
  problemMatcher?: string[];
  isBackground?: boolean;
  presentation?: {
    reveal: 'always' | 'silent' | 'never';
    focus: boolean;
    panel: 'shared' | 'dedicated' | 'new';
  };
  group?: {
    kind: 'build' | 'test' | 'none';
    isDefault?: boolean;
  };
}

export interface RunningTask {
  id: string;
  task: Task;
  processId?: number;
  startTime: Date;
  status: 'running' | 'completed' | 'failed' | 'cancelled';
  output?: string;
  exitCode?: number;
}

class TaskService {
  private tasks: Task[] = [];
  private runningTasks: Map<string, RunningTask> = new Map();
  private defaultBuildTask: string | null = null;

  constructor() {
    this.loadTasks();
    this.loadDefaultBuildTask();
  }

  // Task Management
  addTask(task: Task): void {
    this.tasks.push(task);
    this.saveTasks();
    this.notifyTasksChanged();
  }

  updateTask(taskId: string, updates: Partial<Task>): void {
    const index = this.tasks.findIndex(t => t.id === taskId);
    if (index >= 0) {
      this.tasks[index] = { ...this.tasks[index], ...updates };
      this.saveTasks();
      this.notifyTasksChanged();
    }
  }

  removeTask(taskId: string): void {
    this.tasks = this.tasks.filter(t => t.id !== taskId);
    this.saveTasks();
    this.notifyTasksChanged();
  }

  getTasks(): Task[] {
    return this.tasks;
  }

  getTask(taskId: string): Task | undefined {
    return this.tasks.find(t => t.id === taskId);
  }

  // Running Tasks
  async runTask(taskId: string): Promise<RunningTask> {
    const task = this.getTask(taskId);
    if (!task) {
      throw new Error(`Task ${taskId} not found`);
    }

    const runningTask: RunningTask = {
      id: `run-${Date.now()}`,
      task,
      startTime: new Date(),
      status: 'running',
    };

    this.runningTasks.set(runningTask.id, runningTask);
    this.notifyRunningTasksChanged();

    // Execute task based on type
    try {
      if (task.type === 'npm') {
        await this.runNpmTask(task, runningTask);
      } else if (task.type === 'python') {
        await this.runPythonTask(task, runningTask);
      } else {
        await this.runShellTask(task, runningTask);
      }
    } catch (error) {
      runningTask.status = 'failed';
      runningTask.output = error instanceof Error ? error.message : String(error);
      this.notifyRunningTasksChanged();
    }

    return runningTask;
  }

  async runBuildTask(): Promise<RunningTask | null> {
    if (this.defaultBuildTask) {
      return this.runTask(this.defaultBuildTask);
    }
    
    // Find default build task
    const buildTask = this.tasks.find(t => t.group?.kind === 'build' && t.group?.isDefault);
    if (buildTask) {
      return this.runTask(buildTask.id);
    }

    // Create and run default build task
    const defaultTask: Task = {
      id: 'default-build',
      label: 'Build',
      type: 'shell',
      command: 'npm run build',
      group: { kind: 'build', isDefault: true },
    };
    this.addTask(defaultTask);
    return this.runTask(defaultTask.id);
  }

  async runActiveFile(filePath: string): Promise<RunningTask> {
    const ext = filePath.split('.').pop()?.toLowerCase();
    let command = '';
    let type: Task['type'] = 'shell';

    switch (ext) {
      case 'py':
        command = `python "${filePath}"`;
        type = 'python';
        break;
      case 'js':
      case 'mjs':
        command = `node "${filePath}"`;
        type = 'node';
        break;
      case 'ts':
        command = `ts-node "${filePath}"`;
        type = 'shell';
        break;
      case 'navλ':
      case 'vnc':
        command = `nava run "${filePath}"`;
        type = 'shell';
        break;
      default:
        command = `"${filePath}"`;
    }

    const task: Task = {
      id: `run-active-${Date.now()}`,
      label: `Run ${filePath}`,
      type,
      command,
    };

    return this.runTask(task.id);
  }

  getRunningTasks(): RunningTask[] {
    return Array.from(this.runningTasks.values());
  }

  getRunningTask(taskId: string): RunningTask | undefined {
    return this.runningTasks.get(taskId);
  }

  stopTask(runningTaskId: string): void {
    const runningTask = this.runningTasks.get(runningTaskId);
    if (runningTask) {
      runningTask.status = 'cancelled';
      if (runningTask.processId) {
        // In a real implementation, kill the process
        console.log(`[TaskService] Stopping process ${runningTask.processId}`);
      }
      this.notifyRunningTasksChanged();
    }
  }

  restartTask(runningTaskId: string): Promise<RunningTask> {
    const runningTask = this.runningTasks.get(runningTaskId);
    if (runningTask) {
      this.stopTask(runningTaskId);
      return this.runTask(runningTask.task.id);
    }
    throw new Error(`Running task ${runningTaskId} not found`);
  }

  // Default Build Task
  setDefaultBuildTask(taskId: string): void {
    this.defaultBuildTask = taskId;
    this.saveDefaultBuildTask();
  }

  getDefaultBuildTask(): Task | null {
    if (this.defaultBuildTask) {
      return this.getTask(this.defaultBuildTask);
    }
    return this.tasks.find(t => t.group?.kind === 'build' && t.group?.isDefault) || null;
  }

  // Task Execution
  private async runNpmTask(task: Task, runningTask: RunningTask): Promise<void> {
    // Simulate npm task execution
    runningTask.output = `Running: npm ${task.command}\n`;
    this.notifyRunningTasksChanged();

    // In a real implementation, this would execute the npm command
    await new Promise(resolve => setTimeout(resolve, 1000));
    
    runningTask.status = 'completed';
    runningTask.exitCode = 0;
    runningTask.output += 'Task completed successfully.\n';
    this.notifyRunningTasksChanged();
  }

  private async runPythonTask(task: Task, runningTask: RunningTask): Promise<void> {
    // Simulate Python task execution
    runningTask.output = `Running: python ${task.command}\n`;
    this.notifyRunningTasksChanged();

    // In a real implementation, this would execute the Python command
    await new Promise(resolve => setTimeout(resolve, 1000));
    
    runningTask.status = 'completed';
    runningTask.exitCode = 0;
    runningTask.output += 'Task completed successfully.\n';
    this.notifyRunningTasksChanged();
  }

  private async runShellTask(task: Task, runningTask: RunningTask): Promise<void> {
    // Simulate shell task execution
    runningTask.output = `Running: ${task.command}\n`;
    this.notifyRunningTasksChanged();

    // In a real implementation, this would execute the shell command
    await new Promise(resolve => setTimeout(resolve, 1000));
    
    runningTask.status = 'completed';
    runningTask.exitCode = 0;
    runningTask.output += 'Task completed successfully.\n';
    this.notifyRunningTasksChanged();
  }

  // Persistence
  private saveTasks(): void {
    localStorage.setItem('nava-tasks', JSON.stringify(this.tasks));
  }

  private loadTasks(): void {
    const saved = localStorage.getItem('nava-tasks');
    if (saved) {
      try {
        this.tasks = JSON.parse(saved);
      } catch (error) {
        console.error('[TaskService] Failed to load tasks:', error);
        this.tasks = [];
      }
    } else {
      // Default tasks
      this.tasks = [
        {
          id: 'build',
          label: 'Build',
          type: 'npm',
          command: 'run build',
          group: { kind: 'build', isDefault: true },
        },
        {
          id: 'test',
          label: 'Test',
          type: 'npm',
          command: 'test',
          group: { kind: 'test' },
        },
        {
          id: 'dev',
          label: 'Dev Server',
          type: 'npm',
          command: 'run dev',
          isBackground: true,
        },
      ];
      this.saveTasks();
    }
  }

  private saveDefaultBuildTask(): void {
    if (this.defaultBuildTask) {
      localStorage.setItem('nava-default-build-task', this.defaultBuildTask);
    }
  }

  private loadDefaultBuildTask(): void {
    const saved = localStorage.getItem('nava-default-build-task');
    if (saved) {
      this.defaultBuildTask = saved;
    }
  }

  private notifyTasksChanged(): void {
    const event = new CustomEvent('nava:tasks-changed', {
      detail: { tasks: this.tasks },
    });
    window.dispatchEvent(event);
  }

  private notifyRunningTasksChanged(): void {
    const event = new CustomEvent('nava:running-tasks-changed', {
      detail: { runningTasks: this.getRunningTasks() },
    });
    window.dispatchEvent(event);
  }
}

export const taskService = new TaskService();

