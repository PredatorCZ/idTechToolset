# idTechToolset

idTech Toolset is a collection of modding tools for titles running under idTech engine.

This toolset runs on Spike foundation.

Head to this **[Wiki](https://github.com/PredatorCZ/Spike/wiki/Spike)** for more information on how to effectively use it.

**[Latest Release](https://github.com/PredatorCZ/idTechToolset/releases)**

## Release authenticity

Every release asset will contain corresponding `.sig` file, together with [Sign Key](sign_key.asc) can be used to verify asset via gpg.

Simple usage:

```bash
gpg --import sign_key.asc # Required only once
gpg --verify <asset_name>.sig
```

## Dump DEF

### Module command: def_dump

Dumps animation frame events from `.def` file into Source Engine QC commands.

### Input file patterns: `.def$`

## MD5 Anim to GLTF

### Module command: md5anim_to_gltf

Converts ASCII MD5 animations + input gltf into merged GLTF.

NOTE: The following file patterns apply to `batch.json` which is described [HERE](https://github.com/PredatorCZ/Spike/wiki/Spike---Batching)

### Main file patterns: `.glb$`, `.gltf$`

### Secondary file patterns: `.md5anim$`

## MD5 Mesh to GLTF

### Module command: md5mesh_to_gltf

Converts ASCII MD5 mesh into GLTF.

### Input file patterns: `.md5mesh$`

## License

This toolset is available under GPL v3 license. (See LICENSE.md)\
This toolset uses following libraries:

- Spike, Copyright (c) 2016-2024 Lukas Cone (Apache 2)
