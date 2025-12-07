import React, { useEffect, useRef, useState } from 'react'
import * as THREE from 'three'

type Wheels = { v:number, w:number, left_w:number, right_w:number, constraint_satisfaction:number }

export default function Viewer({wheels}:{wheels:Wheels|null}) {
  const mount = useRef<HTMLDivElement>(null)
  const ringRef = useRef<THREE.Mesh|null>(null)
  const leftRef = useRef<THREE.Mesh|null>(null)
  const rightRef = useRef<THREE.Mesh|null>(null)
  const [renderer] = useState(()=> new THREE.WebGLRenderer({antialias:true}))
  const [scene] = useState(()=> new THREE.Scene())
  const [camera] = useState(()=> new THREE.PerspectiveCamera(60, 16/9, 0.1, 100))

  useEffect(()=>{
    if(!mount.current) return
    const el = mount.current
    renderer.setSize(el.clientWidth, el.clientHeight)
    el.appendChild(renderer.domElement)
    camera.position.set(0,2,4)
    camera.lookAt(0,0,0)

    // ground
    const grid = new THREE.GridHelper(10, 10, 0x113355, 0x102030)
    scene.add(grid)

    // robot body
    const body = new THREE.Mesh(new THREE.CylinderGeometry(0.35,0.35,0.15,32), new THREE.MeshStandardMaterial({color:0x1f2937}))
    body.rotation.x = Math.PI/2
    scene.add(body)

    // LED ring
    const ring = new THREE.Mesh(new THREE.TorusGeometry(0.37, 0.02, 16, 64), new THREE.MeshBasicMaterial({color:0x00ff00}))
    ringRef.current = ring
    scene.add(ring)

    // wheels
    const left = new THREE.Mesh(new THREE.CylinderGeometry(0.06,0.06,0.03,24), new THREE.MeshStandardMaterial({color:0x9ca3af}))
    leftRef.current = left
    left.position.set(0, -0.15, -0.25)
    left.rotation.z = Math.PI/2
    scene.add(left)

    const right = left.clone()
    rightRef.current = right
    right.position.z = 0.25
    scene.add(right)

    // lights
    scene.add(new THREE.AmbientLight(0xffffff, 0.8))
    const d = new THREE.DirectionalLight(0xffffff, 0.6)
    d.position.set(2,2,2)
    scene.add(d)

    let raf = 0
    const tick = ()=>{
      raf = requestAnimationFrame(tick)
      renderer.render(scene, camera)
      // spin wheels according to last wheels event
      if(wheels && leftRef.current && rightRef.current && ringRef.current) {
        leftRef.current.rotation.x -= wheels.left_w * 0.1
        rightRef.current.rotation.x -= wheels.right_w * 0.1
        // LED = satisfaction (green -> red)
        const s = Math.max(0, Math.min(1, wheels.constraint_satisfaction))
        const color = new THREE.Color().setHSL(s*0.33, 1.0, 0.5) // 0.33≈green → 0=red
        ;(ringRef.current.material as THREE.MeshBasicMaterial).color = color
      }
    }
    tick()
    const onResize = ()=>{
      if(!el.isConnected) return
      renderer.setSize(el.clientWidth, el.clientHeight)
      camera.aspect = el.clientWidth/el.clientHeight
      camera.updateProjectionMatrix()
    }
    window.addEventListener('resize', onResize)
    return ()=>{ cancelAnimationFrame(raf); window.removeEventListener('resize', onResize); el.removeChild(renderer.domElement) }
  }, [])

  return <div ref={mount} style={{height:'480px', width:'100%'}} />
}
