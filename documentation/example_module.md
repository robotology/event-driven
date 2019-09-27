# Writing an example module for `event-driven`

You are ready to write your own code that uses the tools in the `event-driven` library to process events. An example module is provided in [example_module](example_module/README.md) that you can use as the basis for writing a module that can be integrated into the `YARP` framework.

The module has the following functionality:

* Reading a `*.ini` file to load configuration options, and setting options via command line arguments.
* Clean exit by capturing `ctrl+c` commands and allowing functions to close ports and clean memory.
* Example `event-driven` ports for reading and writing events.
* A synchronous thread, typically used to show a visualisation or status/debug messages at a readable rate.
* An asynchronous thread, used to read events at sub-millisecond rates and perform processing as required.
* An example CMake file for installing the generated binaries in the install location of `event-driven`, as well as installation of the configuration and application files in locations required by `YARP`.

## How to use the example-module

First, copy the example files to the new location of your project, _e.g._ the same folder where you have cloned `event-driven`.  Assuming a`<path_to_projects>` directory:
```bash
cp -r <path_to_projects>/event-driven/documentation/example-module <path_to_projects>
```

The project can be compiled with modifications:
```bash
cd <path_to_projects>/example-module
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR
make install -j4
```

However, some modifications should be made to personalise the project. The first step would be to change the folder, file and project name:
```bash
cd <path_to_projects>
mv example-module <my-module-name>
mv <my-module-name>/example-module.cpp <my-module-name>/<my-module-name>.cpp
mv example-module.ini <my-module-name>.ini
mv app_example-module.xml app_<my-module-name>.xml
nano <my-module-name> CMakeLists.txt
```
On [line 5](https://github.com/robotology/event-driven/blob/99a1f941141b33266900e034d3e7789d55fd0d99/documentation/example-module/CMakeLists.txt#L5) change the project name to `<my-module-name>`, then save (`ctrl+o`) and exit (`ctrl+x`).
```bash
nano <my-module-name>/<my-module-name>.cpp
```
On line [7](https://github.com/robotology/event-driven/blob/99a1f941141b33266900e034d3e7789d55fd0d99/documentation/example-module/example-module.cpp#L7), [19](https://github.com/robotology/event-driven/blob/99a1f941141b33266900e034d3e7789d55fd0d99/documentation/example-module/example-module.cpp#L19), and [124](https://github.com/robotology/event-driven/blob/99a1f941141b33266900e034d3e7789d55fd0d99/documentation/example-module/example-module.cpp#L124) change the class, constructor and declaration to `<my-module-name>`, then save (`ctrl+o`) and exit (`ctrl+x`).

The module should now be personalised to your processing task.

If you like, you can import the project into your favourite IDE. To do so, _e.g._ for [QtCreator](https://www.qt.io/product) *open a new project* by selecting `<path_to_projects>/<my-module-name>/CmakeLists.txt`. Select the kits `release` and `debug` modifying the build directory to `<path_to_projects>/<my-module-name>/build` and `<path_to_projects>/<my-module-name>/build-debug` respectively. You should now be able to edit, compile, run and debug the module from within QtCreator.