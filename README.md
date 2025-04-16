# Building MAVSDK Library from Source

## Build the C++ Library
This section explains how to build the library along with its unit and integration tests.

### Requirements 
The build requirements are git, cmake, and a compiler such as GCC, Clang, or MSVC.

#### Linux:
##### Ubuntu:

``` 
sudo apt get update
sudo apt-get install build-essential cmake git 

```

##### Fedora: 

```
sudo dnf update
sudo dnf groupinstall "Development Tools" "Development Libraries"
sudo dnf install cmake git
```

##### Arch Linux:

```
sudo pacman -Sy base-devel cmake git
```

## Getting the Source:

### Download the source using git 

``` 
git clone https://github.com/mavlink/MAVSDK.git --recursive
```

### Building:

#### Debug:
To build the MAVSDK C++ Library for development, use the debug build.

There are 2 steps in building a library: Configure and build.

```
sudo cmake -DCMAKE_BUILD_TYPE=Debug -Bbuild/default -S.
sudo cmake --build build/default -j8
```
#### Release:

Once you ship software, make sure to use the release build with optimizations turned on:

##### Linux/macOS: 

```
sudo cmake -Bbuild/default -DCMAKE_BUILD_TYPE=Release -S.
sudo cmake --build build/default -j8
```