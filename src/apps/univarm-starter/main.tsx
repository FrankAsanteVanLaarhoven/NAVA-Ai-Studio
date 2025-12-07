import React from 'react'
import { createRoot } from 'react-dom/client'
import { Toolbar } from './ui/Toolbar'

function App() {
  return (
    <div className="app-shell">
      <div className="left">
        <h2 style={{marginTop:0}}>Univarm × NAVA</h2>
        <p className="muted">Starter toolbar & codegen. Embed this page in your NAVA desktop (iframe or tile).</p>
        <p className="badge">⋋ actions ready</p>
        <hr style={{borderColor:'#1c2633'}}/>
        <Toolbar />
      </div>
      <div className="main">
        <h3>How it works</h3>
        <ol>
          <li>Click <b>Find optimal path (⋋)</b>.</li>
          <li>Choose start/goal; run solver.</li>
          <li>Export code in Rust / C++ / Python / TypeScript.</li>
        </ol>
        <p className="muted">Replace <code>src/engine/mock.ts</code> with your real NAVΛ engine bindings for production.</p>
      </div>
    </div>
  )
}

createRoot(document.getElementById('root')!).render(<App />)
