import React, { useEffect, useState } from 'react'
import Viewer from './viewer/Viewer'
import { solve, submitPath, subscribeSSE } from './api'

export default function App(){
  const [start, setStart] = useState<[number,number]>([0,0])
  const [goal, setGoal] = useState<[number,number]>([5,3])
  const [path, setPath] = useState<{x:number,y:number}[]>([])
  const [wheels, setWheels] = useState<any>(null)
  const [job, setJob] = useState<string|undefined>()

  useEffect(()=>{
    const close = subscribeSSE((kind, data)=>{
      if(kind === 'wheels') setWheels(data)
    })
    return close
  }, [])

  async function onSolve(){
    const r = await solve(start, goal, 200)
    setPath(r.path)
  }
  async function onDrive(){
    if(path.length===0) return
    const r = await submitPath(path)
    setJob(r.job)
  }

  return (
    <div style={{padding:"1rem"}}>
      <div className="flex">
        <div className="panel" style={{minWidth:'320px'}}>
          <div className="row"><span className="badge">Start</span>
            <input value={start[0]} onChange={e=>setStart([+e.target.value, start[1]])}/> 
            <input value={start[1]} onChange={e=>setStart([start[0], +e.target.value])}/>
          </div>
          <div className="row"><span className="badge">Goal</span>
            <input value={goal[0]} onChange={e=>setGoal([+e.target.value, goal[1]])}/> 
            <input value={goal[1]} onChange={e=>setGoal([goal[0], +e.target.value])}/>
          </div>
          <div className="row" style={{marginTop:".5rem", gap:".5rem"}}>
            <button onClick={onSolve}>⋋ Solve Path</button>
            <button onClick={onDrive}>▶ Drive (SSE)</button>
          </div>
          <div style={{marginTop:'.75rem'}}>Job: {job ?? '—'}</div>
          <div style={{marginTop:'.5rem'}}>LED satisfaction <span className="led" style={{background:'#0f0'}}></span> = from SSE</div>
        </div>
        <div className="panel" style={{flex:1}}>
          <Viewer wheels={wheels}/>
        </div>
      </div>
      <div className="panel" style={{marginTop:'1rem'}}>
        <b>Path points:</b> {path.length}
      </div>
    </div>
  )
}
