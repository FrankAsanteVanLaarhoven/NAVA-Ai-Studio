import axios from 'axios'
const BACKEND = import.meta.env.VITE_BACKEND ?? 'http://localhost:8080'

export async function solve(start:[number,number], goal:[number,number], samples=200) {
  const r = await axios.post(`${BACKEND}/api/solve`, { start, goal, samples })
  return r.data as { path: {x:number,y:number,theta?:number}[] }
}

export async function submitPath(path:{x:number,y:number,theta?:number}[]) {
  const r = await axios.post(`${BACKEND}/api/drive/path`, { path })
  return r.data as { job: string }
}

export function subscribeSSE(onEvent:(kind:string,data:any)=>void) {
  const url = `${BACKEND}/api/drive/sse?token=demo-token-123`
  const es = new EventSource(url)
  es.onmessage = (e)=>onEvent('message', JSON.parse(e.data))
  es.addEventListener('wheels', (e:any)=>{
    try { onEvent('wheels', JSON.parse(e.data)) } catch {}
  })
  es.addEventListener('complete', (e:any)=>{
    try { onEvent('complete', JSON.parse(e.data)) } catch {}
  })
  return ()=>es.close()
}
