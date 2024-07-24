<!-- trunk-ignore-all(prettier) -->
<!-- trunk-ignore-all(markdownlint/MD047) -->
# Requirements
- UNIX based OS (didn't test on Windows if this works as it is)
- cmake 3.20 +

# Build instructions
While inside the root folder of this repo:

> _fast build_
```shell
# to build
./build.sh <verbose_option>
# to run
./code/build/main <parameters>
```

or

> _specific build_
```shell
# to build
cd code/build
cmake -D VERBOSE=<verbose_option> ..
make <target_option>
# to run
./main <parameters>
```

## Build options

- verbose_option:
    - **0** : no output (no integrity checks)
    - **10** (or _none_) : normal output (+ integrity checks)
    - **100** : full verbose (+ integrity checks)

- target_options:
    - _none_ : no flag added (default build)
    - **opt** : -O3 flag added (optimized build)
    - **debug** : -g flag added (debug build)

If the build is successful, to run the executable:
```shell
./code/build/main <eventual_parameters>
```

# Parameters

At the moment this software doesn't accept any cli argument.