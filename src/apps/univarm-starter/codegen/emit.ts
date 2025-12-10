import type { Vec3 } from '../engine/nava-engine'

export type Target = 'rust' | 'cpp' | 'python' | 'typescript'

export function emitPath(points: Vec3[], target: Target) {
  switch (target) {
    case 'rust': return emitRust(points)
    case 'cpp': return emitCpp(points)
    case 'python': return emitPython(points)
    case 'typescript': return emitTS(points)
  }
}

function emitRust(points: Vec3[]) {
  const arr = points.map(p=> format(p)).join(", ")
  return `pub fn univarm_path() -> Vec<[f64;3]> {
    vec![${arr}]
}`
}

function emitCpp(points: Vec3[]) {
  const arr = points.map(p=> `{${p.x}, ${p.y}, ${p.z}}`).join(", ")
  return `#include <vector>
#include <array>
std::vector<std::array<double,3>> univarm_path() {
  return { ${arr} };
}`
}

function emitPython(points: Vec3[]) {
  const arr = points.map(p=> `[${p.x}, ${p.y}, ${p.z}]`).join(", ")
  return `def univarm_path():
    return [ ${arr} ]`
}

function emitTS(points: Vec3[]) {
  const arr = points.map(p=> `{x:${p.x}, y:${p.y}, z:${p.z}}`).join(", ")
  return `export type Vec3 = {x:number,y:number,z:number}
export function univarmPath(): Vec3[] { return [ ${arr} ] }`
}

function format(p:{x:number,y:number,z:number}) {
  return `[${p.x}, ${p.y}, ${p.z}]`
}
