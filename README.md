
# Wasm Playground

- [Wasm Playground](#wasm-playground)
  - [Overall](#overall)
  - [NEWS](#news)
  - [Dependancy](#dependancy)
  - [Install Library (emscripten compiler)](#install-library-emscripten-compiler)
  - [Suggestions for Readings](#suggestions-for-readings)
  - [structure](#structure)
  - [Guide](#guide)
  - [Authors](#authors)

---

## Overall

WASM is one of the web standards and It has the benefit of processing speed. According to public DOCS of WASM, WASM is as fast as the native language. I gently suggest using WASM on bottleneck in your algorithm instead of JS.
You can test any algorithm or system based on WASM in this playground.

## NEWS
* [x] Set emscripten(c++,c build toolkit) build env - opencv, pcl, eigen, ceres
* [x] Upload, Calib on React
* [x] Upload, Calib on NEXT
* [x] Upload, Find Plane
* [ ] Upload, jpg Loader Module
## Dependancy

- [Cmake](https://emscripten.org/index.html)
- [emscripten v3.1.26](https://emscripten.org/docs/getting_started/downloads.html)


## Install Library (emscripten compiler)
** Please Keep Build order
1. Eigen
```bash
$ cd thirdparty
$ git clone -b 3.4 https://gitlab.com/libeigen/eigen.git
$ cd eigen
$ mkdir build && cd build
$ emcmake cmake .. && emmake make && make install
```
2. glog
```bash
$ cd thirdparty/glog && mkdir build && cd build
$ emcmake cmake .. && emmake make && make install
```
3. ceres
```bash
$ cd thirdparty/
$ git clone https://github.com/ceres-solver/ceres-solver.git
$ ceres-solver && mkdir build && cd build
$ emcmake cmake ..  && emmake make && make install
```
- opencv v4.1 (**built in thirdparty folder**)
- Flann (**built in thirdparty folder**)
- Boost (**built in thirdparty folder**)
- pcl v1.12 (**built in thirdparty folder**)

**BUILD Test**
```bash
$ wasm_playground/projects/find_plane/core/
$ mkdir build
$ emcmake cmake .. && emmake make && make install
```

**Error Situation**

```bash
$source (emdsk folder)/endsk_env.sh
```

## Suggestions for Readings
[Emscripten DOCS](https://emscripten.org/index.html)

[PCL wasm build reference](https://github.com/luoxuhai/pcl.js)

[WASM official tutorial](https://developer.mozilla.org/en-US/docs/WebAssembly/Loading_and_running)

## structure
```bash
├── Projects
│   ├── ...
│   ├── ...
│   └── (your project name)
│        ├── core
│        │   └── (your CPP or RUST code)
│        │
│        └── js
│            └── (your js code)
│
└── Thirdparty

```

## Guide

- [Notion Page](https://github.com/Superb-AI-Suite/wasm_playground.git)


## Authors

- **Eunsoo Im** - <eslim@superb-ai.com>

