 // Mock editor implementation
 export function createMockEditor(options: any = {}) {
   const defaultOptions = {
     language: 'navlambda',
     theme: 'vs-dark',
     fontSize: 16,
     vncMode: true
   };

   const mergedOptions = { ...defaultOptions, ...options };
   let value = '';
   let disposed = false;
   const contentListeners: Array<() => void> = [];
   const cursorListeners: Array<() => void> = [];
   const actions: any[] = [];
   const completionProviders: any[] = [];

   return {
     // Basic content API
     getValue: () => value,
     setValue: (newValue: string) => { value = newValue; },

     // Options API
     getOption: (option: string) => mergedOptions[option as keyof typeof mergedOptions],
     setOption: (option: string, val: any) => { mergedOptions[option] = val; },
     updateOptions: (newOptions: any) => { Object.assign(mergedOptions, newOptions); },

     // Lifecycle
     dispose: () => { disposed = true; },
     isDisposed: () => disposed,

     // Event handling
     onDidChangeModelContent: (listener: () => void) => {
       contentListeners.push(listener);
       return { dispose: () => {
         const idx = contentListeners.indexOf(listener);
         if (idx >= 0) contentListeners.splice(idx, 1);
       }};
     },
     onDidChangeCursorPosition: (listener: () => void) => {
       cursorListeners.push(listener);
       return { dispose: () => {
         const idx = cursorListeners.indexOf(listener);
         if (idx >= 0) cursorListeners.splice(idx, 1);
       }};
     },
     triggerContentChange: () => {
       // Call listeners synchronously for tests
       contentListeners.slice().forEach(fn => {
         try { fn(); } catch (_) { /* ignore in mock */ }
       });
     },
     triggerCursorChange: () => {
       cursorListeners.slice().forEach(fn => {
         try { fn(); } catch (_) { /* ignore in mock */ }
       });
     },

     // Actions support
     addAction: (action: any) => { actions.push(action); },
     getActions: () => actions.slice(),

     // Completion provider support
     registerCompletionProvider: (selector: string, provider: any) => {
       completionProviders.push(provider);
       return { dispose: () => {
         const idx = completionProviders.indexOf(provider);
         if (idx >= 0) completionProviders.splice(idx, 1);
       }};
     },
     getCompletionProviders: () => completionProviders.slice()
   };
 }

 // Mock terminal session implementation
 export function createMockTerminalSession(options: any = {}) {
   const {
     shell = '/bin/bash',
     cwd = '/workspace',
     rows = 24,
     cols = 80,
     rosEnabled = false
   } = options;

   const id = `mock-session-${Date.now()}-${Math.floor(Math.random() * 1000)}`;
   let status: 'active' | 'closed' = 'active';
   const terminal = {
     rows,
     cols,
     input: [] as string[],
     output: [] as string[]
   };
   const commandHistory: string[] = [];
   const commandHandlers: Record<string, Function> = {};
   const commandResults: Record<string, any> = {};

   function defaultExecuteCommand(command: string) {
     commandHistory.push(command);
     return commandResults[command] || {
       success: true,
       output: `Executed: ${command}`,
       errors: []
     };
   }

   return {
     id,
     status,
     terminal,
     commandHistory,
     executeCommand: defaultExecuteCommand,
     registerCommandHandler: (command: string, handler: Function) => {
       commandHandlers[command] = handler;
     },
     mockExecuteCommand: (command: string, result: any) => {
       commandResults[command] = result;
     },
     mockGetCommandHistory: () => commandHistory.slice(),
     mockClearCommandHistory: () => {
       commandHistory.length = 0;
     },
     dispose: () => {
       status = 'closed';
     }
   };
 }