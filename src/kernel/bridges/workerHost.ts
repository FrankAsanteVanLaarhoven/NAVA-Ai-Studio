/** Mirror kernel bus to a Worker via postMessage. */
export function bridgeWorker(worker: Worker) {
  // Forward messages from worker to main bus namespace (caller wires to Bus).
  return {
    post(channel: string, payload: any) { worker.postMessage({ channel, payload }); },
    listen(onMsg: (channel: string, payload: any) => void) {
      const h = (e: MessageEvent) => {
        const { channel, payload } = e.data || {};
        if (typeof channel === 'string') onMsg(channel, payload);
      };
      worker.addEventListener('message', h);
      return () => worker.removeEventListener('message', h);
    }
  };
}
