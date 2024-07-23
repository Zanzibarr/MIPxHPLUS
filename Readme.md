# Requirements
- UNIX based OS (didn't test on Windows if this works as it is)
- cmake 3.20 +

# Build instructions
While inside the root folder of this repo:
```shell
./build.sh <verbose_option>
```
- <verbose_option>: 0 for no output, 10 for normal output + integrity checks, 100 for full verbose

If anything goes wrong with the building process, a message explaining the problem will appear: please follow the instructions.

If the build is successful, to run the executable:
```shell
./code/build/main <eventual_parameters>
```

# Parameters
At the moment the code doesn't accept any cli argument.