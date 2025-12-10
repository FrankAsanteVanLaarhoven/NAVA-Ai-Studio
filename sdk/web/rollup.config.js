import typescript from 'rollup-plugin-typescript2';
import resolve from 'rollup-plugin-node-resolve';
import dts from 'rollup-plugin-dts';

export default [
  // ESM build
  {
    input: 'src/index.ts',
    output: {
      file: 'dist/index.esm.js',
      format: 'es',
      sourcemap: true,
    },
    plugins: [
      resolve(),
      typescript({
        tsconfig: 'tsconfig.json',
        declaration: false,
      }),
    ],
    external: ['@nava/sdk-wasm'],
  },
  // UMD build for browsers
  {
    input: 'src/index.ts',
    output: {
      file: 'dist/nava-sdk.js',
      format: 'umd',
      name: 'NavaSDK',
      sourcemap: true,
    },
    plugins: [
      resolve(),
      typescript({
        tsconfig: 'tsconfig.json',
        declaration: false,
      }),
    ],
  },
  // Type definitions
  {
    input: 'src/index.ts',
    output: {
      file: 'dist/index.d.ts',
      format: 'es',
    },
    plugins: [dts()],
    external: ['@nava/sdk-wasm'],
  },
];

