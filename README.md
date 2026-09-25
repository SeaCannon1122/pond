# Pond
```
export CMAKE_PREFIX_PATH="$HOME/lib/urdfdom_headers/install:$HOME/lib/urdfdom/install:$CMAKE_PREFIX_PATH"

export POND_CONFIG_PATH="/home/pilot/pond/config/"

export POND_BUNDLE_PATH="$POND_BUNDLE_PATH$(printf ':%s' /home/pilot/pond/build/debug/modules/*/)"
export PYTHONPATH="/home/pilot/pond/build/debug/python:$PYTHONPATH"
```

```
tar -xzf ORBvoc.txt.tar.gz
```