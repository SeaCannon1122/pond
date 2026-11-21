# Pond

Repo has 3 parts
- pond library
- modules
- configurations

## Pond library

The pond library (where robots like N10 or Quac can swim on ;) ) provides a lightweight api for creating modular aplications. It includes a module api for dynamically loading and running the modules as well as a synchronous service api in c that can be used for data distrobution, frame callbacks or anything one desires.

There are also exemplary python bindings one can use if they deem the raw functionallity of the pond_manager as enough for their purposes.

## Modules

This is a codebase of all the modules that were implemented using this library. Most of them are drivers or provide logic for one of the robots they were written for.


## Configurations

This section contains configuration, data and launch among other files for some of the robots mentioned above.

```
export CMAKE_PREFIX_PATH="$HOME/lib/urdfdom_headers/install:$HOME/lib/urdfdom/install:$CMAKE_PREFIX_PATH"

export POND_QUAC_CONFIG_PATH="/home/pilot/pond/configs/quac/"
export POND_IDEFIX_CONFIG_PATH="/home/pilot/pond/configs/idefix/"

export POND_BUNDLE_PATH="/home/pilot/pond/build/debug/pond_bundles"
export PYTHONPATH="/home/pilot/pond/build/debug/python:$PYTHONPATH"
```

```
tar -xzf ORBvoc.txt.tar.gz
```