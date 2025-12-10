import React from 'react'
import manifest from '../prefixes/nava-prefixes.json'
import { solveOptimalPath, getAvailableInterpreters, type InterpreterType } from '../engine/nava-engine'
import { emitPath, type Target } from '../codegen/emit'

function Card(props:{title:string, children:React.ReactNode}) {
  return <div style={{border:'1px solid #1c2633', borderRadius:12, padding:12, marginBottom:12}}>
    <div style={{fontWeight:600, marginBottom:8}}>{props.title}</div>
    {props.children}
  </div>
}

export function Toolbar() {
  const [sx, setSx] = React.useState(0)
  const [sy, setSy] = React.useState(0)
  const [sz, setSz] = React.useState(0)
  const [gx, setGx] = React.useState(5)
  const [gy, setGy] = React.useState(5)
  const [gz, setGz] = React.useState(5)
  const [code, setCode] = React.useState<string>('')
  const [status, setStatus] = React.useState<string>('Idle')
  const [points, setPoints] = React.useState<{x:number,y:number,z:number}[]|null>(null)
  const [interpreter, setInterpreter] = React.useState<InterpreterType>('auto')
  const [availableInterpreters, setAvailableInterpreters] = React.useState<InterpreterType[]>(['browser-sim'])
  const [energyLandscape, setEnergyLandscape] = React.useState(true)

  // Load available interpreters on mount
  React.useEffect(() => {
    getAvailableInterpreters().then(setAvailableInterpreters)
  }, [])

  async function run() {
    setStatus('Solving with NAVΛ…')
    try {
      const res = await solveOptimalPath({ 
        start:{x:sx,y:sy,z:sz}, 
        goal:{x:gx,y:gy,z:gz}, 
        samples:48,
        interpreter,
        energyLandscape,
      })
      setPoints(res.points)
      const metadata = res.metadata 
        ? ` (${res.metadata.interpreter}, ${res.metadata.executionTime.toFixed(1)}ms)`
        : ''
      setStatus(`OK — cost ≈ ${res.cost.toFixed(3)}, energy ≈ ${res.energy?.toFixed(3) || res.cost.toFixed(3)}${metadata}`)
      setCode('') // clear stale code
    } catch (error: any) {
      setStatus(`Error: ${error.message}`)
    }
  }

  function exportCode(target: Target) {
    if (!points) return
    const out = emitPath(points, target)
    setCode(out)
  }

  return <div>
    <Card title="NAVΛ Engine Configuration">
      <div style={{display:'flex', gap:8, alignItems:'center', marginBottom:8}}>
        <label style={{fontSize:12, color:'#88a2bf', minWidth:80}}>Interpreter:</label>
        <select 
          value={interpreter} 
          onChange={e=>setInterpreter(e.target.value as InterpreterType)}
          style={{
            flex:1, background:'#0e141c', color:'#e6f1ff', 
            border:'1px solid #1c2633', borderRadius:8, padding:'6px 8px', fontSize:12
          }}
        >
          <option value="auto">Auto (Best Available)</option>
          {availableInterpreters.map(interp => (
            <option key={interp} value={interp}>
              {interp === 'tauri-backend' ? '🦀 Tauri Backend (Rust)' :
               interp === 'python' ? '🐍 Python (Pyodide)' :
               interp === 'wasm' ? '⚡ WebAssembly' :
               interp === 'browser-sim' ? '🌐 Browser Simulation' : interp}
            </option>
          ))}
        </select>
      </div>
      <label style={{display:'flex', gap:8, alignItems:'center', fontSize:12, color:'#88a2bf'}}>
        <input 
          type="checkbox" 
          checked={energyLandscape} 
          onChange={e=>setEnergyLandscape(e.target.checked)}
          style={{cursor:'pointer'}}
        />
        <span>Use VNC Energy Landscape Optimization</span>
      </label>
    </Card>

    <Card title="One-click actions">
      <button onClick={run} style={btn()}>Find optimal path (⋋)</button>
      <div style={{fontSize:12, color:'#88a2bf', marginTop:6}}>
        Prefix: <code>λopt</code> — manifest-backed (see <code>src/prefixes/nava-prefixes.json</code>).
      </div>
      <div style={{display:'grid', gridTemplateColumns:'repeat(3,1fr)', gap:8, marginTop:8}}>
        <Num label="Start X" v={sx} set={setSx}/>
        <Num label="Start Y" v={sy} set={setSy}/>
        <Num label="Start Z" v={sz} set={setSz}/>
        <Num label="Goal X" v={gx} set={setGx}/>
        <Num label="Goal Y" v={gy} set={setGy}/>
        <Num label="Goal Z" v={gz} set={setGz}/>
      </div>
      <div style={{marginTop:8}}><span className="badge">{status}</span></div>
    </Card>

    <Card title="Export">
      <div style={{display:'flex', gap:8, flexWrap:'wrap'}}>
        <button onClick={()=>exportCode('rust')} style={btn()}>Emit Rust</button>
        <button onClick={()=>exportCode('cpp')} style={btn()}>Emit C++</button>
        <button onClick={()=>exportCode('python')} style={btn()}>Emit Python</button>
        <button onClick={()=>exportCode('typescript')} style={btn()}>Emit TypeScript</button>
      </div>
      <textarea value={code} readOnly rows={12} style={{
        width:'100%', marginTop:8, background:'#0e141c', color:'#cfe5ff',
        border:'1px solid #1c2633', borderRadius:8, padding:10, fontFamily:'ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas'
      }}/>
    </Card>

    <Card title="Prefix manifest (excerpt)">
      <pre style={{whiteSpace:'pre-wrap', margin:0, fontSize:12, color:'#a9c1db'}}>
        {JSON.stringify(manifest.groups[0], null, 2)}
      </pre>
    </Card>
  </div>
}

function Num({label, v, set}:{label:string, v:number, set:(n:number)=>void}) {
  return <label style={{display:'grid', gap:4}}>
    <span style={{fontSize:12, color:'#88a2bf'}}>{label}</span>
    <input type="number" value={v} onChange={e=>set(parseFloat(e.target.value))}
      style={{background:'#0e141c', color:'#e6f1ff', border:'1px solid #1c2633', borderRadius:8, padding:'6px 8px'}}
    />
  </label>
}

function btn() {
  return {
    background:'#0e1a12', color:'#9effc8', border:'1px solid #244a33',
    borderRadius:10, padding:'8px 12px', cursor:'pointer'
  } as React.CSSProperties
}
