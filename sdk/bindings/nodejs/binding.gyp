{
  "targets": [
    {
      "target_name": "nava_sdk",
      "sources": [
        "src/nava_sdk.cc"
      ],
      "include_dirs": [
        "<!@(node -e \"require('node-addon-api').include\")",
        "../native/src"
      ],
      "defines": [
        "NAPI_VERSION=8"
      ],
      "cflags!": [ "-fno-exceptions" ],
      "cflags_cc!": [ "-fno-exceptions" ],
      "xcode_settings": {
        "GCC_ENABLE_CPP_EXCEPTIONS": "YES",
        "CLANG_CXX_LIBRARY": "libc++",
        "MACOSX_DEPLOYMENT_TARGET": "10.7"
      },
      "msvs_settings": {
        "VCCLCompilerTool": {
          "ExceptionHandling": 1
        }
      }
    }
  ]
}

