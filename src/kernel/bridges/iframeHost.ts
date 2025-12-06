export function bridgeIframe(frame: HTMLIFrameElement) {
  const target = frame.contentWindow!;
  return {
    post(channel: string, payload: any) { target.postMessage({ channel, payload }, "*"); },
    listen(onMsg: (channel: string, payload: any) => void) {
      const h = (e: MessageEvent) => {
        const { channel, payload } = e.data || {};
        if (typeof channel === 'string') onMsg(channel, payload);
      };
      window.addEventListener('message', h);
      return () => window.removeEventListener('message', h);
    }
  };
}
